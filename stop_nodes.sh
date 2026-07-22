#!/bin/bash
# Deprecated — superseded by ./hauv.sh
# Kept as a wrapper so old habits keep working.  See:  ./hauv.sh help
exec "$(dirname "$0")/hauv.sh" stop
