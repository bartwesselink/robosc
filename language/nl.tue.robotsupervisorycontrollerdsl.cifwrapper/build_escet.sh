#!/bin/sh
echo "Cloning ESCET from repository..."

REPO="$1"
SHA="$2"

rm -rf escet-repo
mkdir -p escet-repo
git clone --depth 1 $REPO escet-repo
cd escet-repo

git fetch --depth=1 origin $SHA
git checkout $SHA

./mvn_escet.sh -DskipTests clean verify
rm -rf product/
rm -rf .git