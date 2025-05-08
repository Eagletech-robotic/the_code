#!/bin/bash
set -euxo pipefail

cppcheck -I falcon/Core/Inc/ falcon/Core


