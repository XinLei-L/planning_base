#### Lifecycle_Node

这个项目主要是学习ROS2中的生命周期节点

项目src目录内容有以下：

- lifecycle_node_demo：将节点设置成生命周期节点，并在其中加入topic发布，控制发布话题节点
- lifecycle_node_service：在lifecycle_node_demo中，程序运行是处于unconfigure状态的。要想启动发布话题节点，还需要手动configure和activate（或者直接托管给生命周期节点管理器）。对于手动设置，在工程中是不现实的，基本上都是通过程序自动设置。而这份程序就是用来自动设置节点的状态

