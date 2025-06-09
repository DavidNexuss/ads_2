#!/bin/bash
git submodule update --init
./scripts/applypatch.sh
./scripts/make.sh
