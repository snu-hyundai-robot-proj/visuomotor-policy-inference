#!/usr/bin/env bash
# Recover the RIGHT robot stack (down -> up, fix common stuck states). See restack.sh.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/restack.sh" right
