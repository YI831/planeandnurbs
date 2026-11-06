关于 Easy3DViewer 的说明

文件夹说明：

3rd_party
	easy3d 依赖的第三方库（包括Eigen）。我把 opennurbs 和 sisl 也加进去了，因为这样配环境比较方便，不过编译的时候会带着sisl一起编译，会慢一些。所以建议你把sisl去掉， 然后在 Easy3DViewer 中单独设置 set(SISL_INCLUDE_DIR 这里写你的sisl路径)

cmake
	可以不用管

docs
	easy3d 自带的几个文档

easy3d
	easy3d的代码

models
	可以把你的求交案例放在这个文件夹里面，方便读取

myviewer
	界面的实现代码。SplineObject 文件提供了 Nurbs 的可视化；在 myviewer 文件你可以修改代码对界面进行编辑

NurbsPlaneInt
	可以把你的求交代码放在这个文件夹里面，替换原本的 test 文件，另外修改一下CMakeList

resources
	easy3d 渲染用到的一些文件，可以不管。其中 resources/data 里有一些可以读取的模型

tutorials
	与主要程序无关，有哪些功能不会写可以参考里面的代码