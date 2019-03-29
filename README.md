# device-sdk-go中golang内嵌libiio API交叉编译到ZC702
  1.运行setcclibiio.sh配置环境
  
  2.修改makefile CGO_ENABLE=1
  
  3.修改go env  
  
  export GOARCH=arm
  
  export CC=arm-linux-gnueabihf-gcc
  
  4.内嵌C代码中添加
  
  #cgo CFLAGS: -Wall
  
  #cgo LDFLAGS: -L /usr/lib -lxml2 -lz -pthread

  
