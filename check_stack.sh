#!/bin/bash
set -euxo pipefail

find . -name "*.su" -exec cat {} + | perl -pe 's{^(.*\))(\s+)(\d+)(\s+)(static.*)}{$3 $1$2$4$5}' | sort -n | tail -n 15 
