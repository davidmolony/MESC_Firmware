#! /bin/bash

# create a temp directory & build book into it
tmpdir=$(mktemp -d /tmp/bookXXX)
mdbook build -d ${tmpdir}
echo "build directory is: ${tmpdir}"

# go to temp directory
cd ${tmpdir}

# indicate that GitHub should not interpret this as a Jekyll site, i.e.
# it's a static site.
touch .nojekyll

# create new git repo
git init

# ignore ~ files from emacs
echo '*~' > .gitignore

# add all fles in branch 'main'
git add .
git branch -M main

# commit
git commit -m "latest"

# push to github pages
git push -f https://github.com/davidmolony/MESC_Firmware.git main:gh-pages
