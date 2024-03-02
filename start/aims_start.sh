#! /bin/bash

# set variables
SCRIPT=$(readlink -f $0)
SCRIPTPATH=$(dirname $SCRIPT)
cd "$SCRIPTPATH"

TARGET=$1

# remove the old sink
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln $TARGET .tmuxinator.yml

# start tmuxinator
tmuxinator start -p $TARGET
