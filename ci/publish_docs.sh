#!/bin/bash

set -x

DOCS_DIR=pages/

if [[ ${CI} -ne "" ]] || [[ ${TRAVIS} -ne "" ]]; then
    GITHUB=https://germangb:$TOKEN@github.com/germangb/newton-rs.git
else
    GITHUB=git@github.com:germangb/newton-rs.git
fi

cargo doc --all-features
rm -rf $DOCS_DIR
cp -r target/doc $DOCS_DIR

cat << EOF > $DOCS_DIR/index.html
<!DOCTYPE html>
<html>
<head>
<meta http-equiv="refresh" content="0; url=newton/index.html" />
</head>
<body>
    You are being redirected to the <a href="newton/index.html">documentation page</a>...
</body>
</html>
EOF

git -C $DOCS_DIR init && \
    git -C $DOCS_DIR remote add origin $GITHUB && \
    git -C $DOCS_DIR checkout -b gh-pages && \
    git -C $DOCS_DIR add -A && \
    git -C $DOCS_DIR commit -m "Publish docs" > /dev/null && \
git -C $DOCS_DIR push origin gh-pages --force --quiet
