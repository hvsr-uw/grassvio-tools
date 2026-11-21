"""Feather nearby source boundaries while retaining untouched interior pixels."""

import cv2
import numpy as np


class SeamBlender:
    """Track the best available alternative to each hard-owner pixel."""

    def __init__(self, radius):
        self.radius = radius
        self.owner = self.alternative = self.alt_score = self.alt_color = None
        self.next_id = 0

    def resize(self, height, width, offset_x, offset_y):
        def grow(previous, shape, fill, dtype):
            canvas = np.full(shape, fill, dtype=dtype)
            if previous is not None:
                h, w = previous.shape[:2]
                canvas[offset_y:offset_y+h, offset_x:offset_x+w] = previous
            return canvas

        shape = (height, width)
        self.owner = grow(self.owner, shape, -1, np.int32)
        self.alternative = grow(self.alternative, shape, -1, np.int32)
        self.alt_score = grow(self.alt_score, shape, 0, np.float32)
        self.alt_color = grow(self.alt_color, (*shape, 3), 0, np.uint8)

    def update_sources(self, region, hard, scores, warped, score, valid, take):
        """Update per-pixel sources before overwriting the previous winners."""
        if not take.any():
            return
        owner = self.owner[region]
        alternative = self.alternative[region]
        alt_score = self.alt_score[region]
        alt_color = self.alt_color[region]
        # A displaced winner is the best alternative. Never reuse its blended color.
        displaced = take & (owner >= 0)
        alternative[displaced] = owner[displaced]
        alt_score[displaced] = scores[displaced]
        alt_color[displaced] = hard[displaced]

        runner_up = valid & ~take & (score > alt_score)
        alternative[runner_up] = self.next_id
        alt_score[runner_up] = score[runner_up]
        alt_color[runner_up] = warped[runner_up]
        owner[take] = self.next_id
        self.next_id += 1

    def render(self, hard, display, x, y, width, height):
        """Refresh the changed area and its seam halo using the hard source colors."""
        radius = self.radius
        full_h, full_w = hard.shape[:2]
        x0, y0 = max(0, x - radius), max(0, y - radius)
        x1, y1 = min(full_w, x + width + radius), min(full_h, y + height + radius)
        hx0, hy0 = max(0, x0 - radius), max(0, y0 - radius)
        hx1, hy1 = min(full_w, x1 + radius), min(full_h, y1 + radius)
        target = np.s_[y0:y1, x0:x1]
        window = np.s_[hy0:hy1, hx0:hx1]
        local = np.s_[y0-hy0:y1-hy0, x0-hx0:x1-hx0]
        output = display[target]
        output[:] = hard[target]
        owners = self.owner[window]
        alternatives = self.alternative[target]
        for other_id in np.unique(alternatives[alternatives >= 0]):
            other_owner = owners == other_id
            if not other_owner.any():
                continue
            distance = cv2.distanceTransform((~other_owner).astype(np.uint8),
                                             cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[local]
            near = (alternatives == other_id) & (distance <= radius)
            # Smoothstep from half the neighboring color at the boundary to
            # zero at the outer edge. Both colors are supported at these pixels.
            t = (distance[near] - .5) / radius
            alpha = (.5 * (1. - t*t*(3. - 2.*t)))[:, None]
            output[near] = np.rint(output[near] * (1 - alpha)
                                   + self.alt_color[target][near] * alpha).astype(np.uint8)


def measure_seams(hard, blended, owner):
    """Compare color jumps across the same four-connected owner boundaries."""
    before = np.zeros(owner.shape, np.float32)
    after = before.copy()
    edge_count = 0
    for a, b in ((np.s_[:, :-1], np.s_[:, 1:]), (np.s_[:-1, :], np.s_[1:, :])):
        edge = (owner[a] >= 0) & (owner[b] >= 0) & (owner[a] != owner[b])
        edge_count += int(edge.sum())
        before[a][edge] += np.abs(hard[a][edge].astype(np.float32) - hard[b][edge]).mean(axis=1)
        after[a][edge] += np.abs(blended[a][edge].astype(np.float32) - blended[b][edge]).mean(axis=1)
    edited = np.any(hard != blended, axis=2)
    if np.any(edited & (owner < 0)):
        raise ValueError("seam blending wrote beyond source coverage")
    covered = int(np.count_nonzero(owner >= 0))
    return {
        "covered_pixels": covered,
        "changed_pixels": int(edited.sum()),
        "owner_boundary_edges": edge_count,
        "mean_color_jump_before": float(before.sum() / edge_count) if edge_count else None,
        "mean_color_jump_after": float(after.sum() / edge_count) if edge_count else None,
    }, before - after


def seam_preview(hard, blended, improvement):
    """Show the same crop in both images, centered on a measurable change."""
    size = min(240, *improvement.shape)
    local_gain = cv2.boxFilter(np.maximum(improvement, 0), -1, (size, size),
                               normalize=False, borderType=cv2.BORDER_CONSTANT)
    height, width = improvement.shape
    left, right = size // 2, size - size // 2
    y, x = np.unravel_index(local_gain[left:height-right+1, left:width-right+1].argmax(),
                            (height - size + 1, width - size + 1))
    panel = np.full((size + 45, size * 2 + 12, 3), 248, np.uint8)
    for offset, label, photo in (
        (0, "Hard join", hard), (size + 12, "Thin seam blend", blended),
    ):
        panel[45:, offset:offset+size] = photo[y:y+size, x:x+size]
        cv2.putText(panel, label, (offset + 7, 29), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (25, 25, 25), 1, cv2.LINE_AA)
    return panel, [int(x), int(y), size, size]
