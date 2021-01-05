#!/bin/bash
die(){ echo "Coudn't $*" >&2; exit 1; }

echo -e "\e[36mStarting $0\e[39m: install deps"

npm install -g gitbook-cli || die "install gitbook-cli"
gitbook install || die "run gitbook install"

echo -e "\e[36m$0\e[39m: build"

gitbook build -d || die "Build the documentation in HTML"

echo -e "\e[36m$0\e[32m: BUILD SUCCEEDED!\e[39m"

