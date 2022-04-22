#!/bin/sh
echo "Cloning ESCET from repository..."

REPO="$1"
BRANCH="$2"

rm -rf escet-repo
mkdir -p escet-repo
git clone --depth 1 $REPO escet-repo -b $BRANCH

cd escet-repo
./mvn_escet.sh -DskipTests clean verify
rm -rf product/
rm -rf .git