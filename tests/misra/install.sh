#!/bin/bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
: "${CPPCHECK_DIR:=$DIR/cppcheck/}"

if [ ! -d "$CPPCHECK_DIR" ]; then
  git clone https://github.com/danmar/cppcheck.git $CPPCHECK_DIR
fi

cd $CPPCHECK_DIR
git fetch --all
git checkout 65da79394471784641568dc4ac5da5466599809d
git -c user.name=a -c user.email="a@b.c" cherry-pick 7199dde1618b5166735f07619dcdb9f4eafdb557
make clean
make MATCHCOMPILTER=yes CXXFLAGS="-O2" -j8
