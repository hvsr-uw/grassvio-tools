# Julius's pushed incomplete code — review attempt

Date: 2026-11-06. Julius wrote in the team chat (Nov 5, 11:09 PM): "i
pushed my incompeleted code already" and "i think ill let you guys know
tomorrow before or on meetings".

## Result: could not read the code

The team repo Donkey0322/GrassVIO is not reachable with the stored
credential. Both the repo endpoint and the branches endpoint return the
same error:

- GET https://api.github.com/repos/Donkey0322/GrassVIO -> HTTP Error 404: Not Found
- GET https://api.github.com/repos/Donkey0322/GrassVIO/branches?per_page=100 -> HTTP Error 404: Not Found

The stored credential does work: the same credential lists
Donkey0322's public repos (12 repos, no GrassVIO among them) and reads
hvsr-uw/grassvio-tools normally. A GitHub search for "GrassVIO" returns
only hvsr-uw/grassvio-tools. So Donkey0322/GrassVIO is either private
with no access granted to this credential, or it does not exist under
that exact name. Either way it cannot be reviewed from here.

## What was checked instead

- hvsr-uw/grassvio-tools, last 15 commits: all authored by Harsh
  Rathore (Nov 2 and Nov 6, render v2 and loop-closure work). No
  commits by Julius, no other branches besides main.
- Donkey0322 (Don Lee) is an active account: other public repos updated
  Nov 6 (JobRadar-AI, jobradar-personal). That does not imply anything
  about GrassVIO access.

## What this means

Julius most likely pushed to the private team repo Donkey0322/GrassVIO,
which this credential cannot see. No summary of what the code does, what
is incomplete, or whether it runs can be given without access. To
review it, Julius (or Don, as repo owner) needs to grant the hvsr-uw
account read access, or Julius needs to push the same code somewhere
readable, for example a branch on hvsr-uw/grassvio-tools.

## Open question for Julius

Which repo and branch did you push to, and can you add read access for
the rest of the team? Until then nobody else can look at the code.
