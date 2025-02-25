<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a >
    <img src="logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Balanced-bicycle</h3>

  <p align="center">
    十七届恩智浦智能车竞赛单车组代码。

  </p>
</div>







<!-- ABOUT THE PROJECT -->
## 关于项目
1.编写所有代码在 `CODE`文件夹中。

2.文件内容：
`ANO_DT.c`为匿名上位机通讯协议。可实现与匿名上位机进行协议收发。实现数据可视化。可根据自己需要修改数据发送数目。

`Key.c`为按键读取代码，可根据自己按键GPIO口修改引脚。内含按键消抖代码。

`KeySettingMenu.c`为多级菜单代码。可根据自己需要移植。可拓展多级菜单。

`PID.c`为PID算法的参数文件

`algorithm.c`内含各种算法。具体有：一阶互补滤波、陀螺仪零飘校准、动态零点、线性归一化、滑动滤波、直立串级PID（角速度环-角度环-速度环）

`control.c`为单车运动控制文件。内容包含后轮电机的增量式PID控制、电磁循迹PID控制、差比和算法、赛道电磁元素处理等。可精确识别三叉元素。

`my_tft.c`为自己编写的LCD屏幕的显示库与菜单配合使用。可实现选项背景高亮等功能。


<!-- LICENSE -->
## License

根据项目许可证分发。有关更多信息，请参阅 `LICENSE.txt`



<!-- ACKNOWLEDGMENTS -->



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

