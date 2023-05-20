INSTALL_DIR=/etc/sophia/
rm -rf $INSTALL_DIR
mkdir -p $INSTALL_DIR
cp -r python $INSTALL_DIR
cp -r scripts $INSTALL_DIR
ln -s $INSTALL_DIR/scripts/sophia /bin/
