#!/bin/bash
# Deprecated — superseded by ./hauv.sh
# Kept as a wrapper so rov_nodes.service (and old habits) keep working.
# For modes/diagnostics use:  ./hauv.sh help
exec "$(dirname "$0")/hauv.sh" start "$@"
