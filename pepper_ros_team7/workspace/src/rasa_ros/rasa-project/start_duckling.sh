#!/bin/bash

BOT_DIR=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "$BOT_DIR/duckling"

stack exec duckling-example-exe