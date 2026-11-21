"""Recording-specific RealSense calibration and read-only synchronized access.

Projection follows librealsense src/rs.cpp. Recorded tf/ref transforms map
the common reference frame into each stream; SDK rotations are column-major.
"""
from pathlib import Path
import sqlite3
import struct
import sys

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"bag-exploration"))
from explore_bag import COLOR,DEPTH,GYRO,ACCEL,parse_image,parse_imu,parse_ros_string


def fields(text):
    return dict(piece.split("=",1) for piece in text.split(";") if "=" in piece)


def camera(text):
    item=fields(text)
    result={key:float(item[key]) for key in ("fx","fy","ppx","ppy")}
    result.update(width=int(item["width"]),height=int(item["height"]),
                  model=item["model"],coeffs=[float(v) for v in item["coeffs"].split(",")])
    if result["model"] not in ("None","Brown Conrady","Inverse Brown Conrady"):
        raise ValueError("unsupported recorded distortion model")
    result["K"]=np.array([[result["fx"],0,result["ppx"]],
                          [0,result["fy"],result["ppy"]],[0,0,1.]])
    return result


def extrinsic(text):
    item=fields(text)
    r=np.array([float(v) for v in item["rotation"].split(",")]).reshape(3,3,order="F")
    if not np.allclose(r.T@r,np.eye(3),atol=2e-5) or np.linalg.det(r)<0:
        raise ValueError("invalid recorded extrinsic rotation")
    u,_,vt=np.linalg.svd(r)
    return u@vt,np.array([float(v) for v in item["translation"].split(",")])


def rays(pixels,cal):
    p=np.asarray(pixels,float).reshape(-1,2)
    original=(p-[cal["ppx"],cal["ppy"]])/[cal["fx"],cal["fy"]]
    xy=original.copy()
    k1,k2,p1,p2,k3=cal["coeffs"]
    if any(cal["coeffs"]):
        for _ in range(10):
            x,y=xy.T
            r2=x*x+y*y
            radial=1+((k3*r2+k2)*r2+k1)*r2
            if cal["model"]=="Inverse Brown Conrady":
                tx,ty=x*radial,y*radial
            elif cal["model"]=="Brown Conrady":
                tx,ty=x,y
            else:
                raise ValueError("nonzero coefficients with unsupported model")
            tangent=np.column_stack((2*p1*tx*ty+p2*(r2+2*tx*tx),
                                     2*p2*tx*ty+p1*(r2+2*ty*ty)))
            xy=(original-tangent)/radial[:,None]
    return np.column_stack((xy,np.ones(len(xy))))


def project(points,cal):
    p=np.asarray(points,float).reshape(-1,3)
    x,y=(p[:,:2]/p[:,2:]).T
    r2=x*x+y*y
    k1,k2,p1,p2,k3=cal["coeffs"]
    radial=1+((k3*r2+k2)*r2+k1)*r2
    scaled_x,scaled_y=x*radial,y*radial
    if cal["model"]=="Inverse Brown Conrady":
        x,y=scaled_x,scaled_y
    dx=2*p1*x*y+p2*(r2+2*x*x)
    dy=2*p2*x*y+p1*(r2+2*y*y)
    return np.column_stack(((scaled_x+dx)*cal["fx"]+cal["ppx"],
                             (scaled_y+dy)*cal["fy"]+cal["ppy"]))


class DepthPairingError(ValueError):
    """A colour frame has no depth observation within the pairing limit."""


class Recording:
    def __init__(self,path):
        self.path=Path(path)
        self.db=sqlite3.connect(self.path.resolve().as_uri()+"?mode=ro",uri=True)
        self.db.execute("PRAGMA query_only=ON")
        self.topics={name:(tid,kind) for tid,name,kind in self.db.execute("SELECT id,name,type FROM topics")}
        self.metadata={}
        for name,(tid,kind) in self.topics.items():
            if kind=="std_msgs/msg/String" and any(w in name for w in ("camera_info","/tf/","Depth_Units/value")):
                row=self.db.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1",(tid,)).fetchone()
                if row:self.metadata[name]=parse_ros_string(row[0])
        self.color=camera(self.metadata[COLOR.replace("image/data","camera_info")])
        self.depth=camera(self.metadata[DEPTH.replace("image/data","camera_info")])
        self.depth_unit=float(self.metadata["/device_0/sensor_0/option/Depth_Units/value"])
        self.rows={}
        for name in (COLOR,DEPTH):
            tid=self.topics[name][0]
            result=list(self.db.execute("SELECT rowid,substr(data,1,16) FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)))
            self.rows[name]=[(rowid,struct.unpack_from("<i",b,4)[0]*1_000_000_000+struct.unpack_from("<I",b,8)[0]) for rowid,b in result]
        self.color_times=np.array([t for _,t in self.rows[COLOR]],dtype=np.int64)
        self.depth_times=np.array([t for _,t in self.rows[DEPTH]],dtype=np.int64)
        self.color_from_depth=self.relative(COLOR,DEPTH)
        self.color_from_gyro=self.relative(COLOR,GYRO)
        self.color_from_accel=self.relative(COLOR,ACCEL)
        yy,xx=np.mgrid[:self.depth["height"],:self.depth["width"]]
        self.depth_rays=rays(np.column_stack((xx.ravel(),yy.ravel())),self.depth)

    def relative(self,target,source):
        def tf(topic):
            return extrinsic(self.metadata[topic.replace("image/data","tf/ref_0").replace("imu/data","tf/ref_0")])
        rt,tt=tf(target);rs,ts=tf(source)
        r=rt@rs.T
        return r,tt-r@ts

    def image(self,topic,index):
        rowid=self.rows[topic][index][0]
        blob=self.db.execute("SELECT data FROM messages WHERE rowid=?",(rowid,)).fetchone()[0]
        p=parse_image(blob)
        if topic==COLOR:
            if p["encoding"]!="rgb8":raise ValueError("rgb8 input required")
            return cv2.cvtColor(np.frombuffer(p["data"],np.uint8).reshape(p["height"],p["width"],3),cv2.COLOR_RGB2BGR)
        if p["encoding"] not in ("16UC1","mono16"):raise ValueError("16-bit depth required")
        return np.frombuffer(p["data"],dtype="<u2").reshape(p["height"],p["width"])*self.depth_unit

    def depth_pair_for_color(self,index,max_offset_ms=20.):
        if not np.isfinite(max_offset_ms) or max_offset_ms < 0:
            raise ValueError("depth pairing limit must be finite and nonnegative")
        if not len(self.depth_times):
            raise DepthPairingError("recording has no depth observations")
        time=self.color_times[index]
        depth_index=int(np.argmin(np.abs(self.depth_times-time)))
        offset=abs(int(self.depth_times[depth_index])-int(time))/1e6
        if offset>max_offset_ms:
            raise DepthPairingError("colour/depth offset exceeds pairing limit")
        return depth_index,offset

    def cloud_for_color(self,index,max_offset_ms=20.):
        depth_index,offset=self.depth_pair_for_color(index,max_offset_ms)
        z=self.image(DEPTH,depth_index).ravel()
        good=(z>.05)&(z<6.)
        r,t=self.color_from_depth
        points=(self.depth_rays[good]*z[good,None])@r.T+t
        positive=points[:,2]>.05
        points=points[positive]
        uv=np.rint(project(points,self.color)).astype(int)
        h,w=self.color["height"],self.color["width"]
        valid=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        uv,points=uv[valid],points[valid]
        # Z-buffer: only the closest measured surface owns a colour pixel.
        order=np.argsort(points[:,2],kind="stable")
        pixels=uv[order,1]*w+uv[order,0]
        _,first=np.unique(pixels,return_index=True)
        cloud=np.full((h*w,3),np.nan)
        cloud[pixels[first]]=points[order[first]]
        return cloud.reshape(h,w,3),offset

    def imu(self,topic):
        tid=self.topics[topic][0]
        parsed=[parse_imu(blob) for (blob,) in self.db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))]
        key="angular_velocity" if topic==GYRO else "linear_acceleration"
        return np.array([p["stamp_ns"] for p in parsed],dtype=np.int64),np.array([p[key] for p in parsed])

    def close(self):
        self.db.close()
