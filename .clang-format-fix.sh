#!/bin/bash

source .clang-format-common.sh

for f in ${src_files}; do
  $clang_format -style=file -i $f
done
