1. 在主目录下配置程序运行库的环境
   gedit ~/.bashrc
   在该文件最后加入：
   export LD_LIBRARY_PATH=/home/user/Desktop/HandGuidingExample/lib/lib32
注意：   home/user/Desktop是相对路径，应用时以实际的路径为准。
   保存并并退出。
   
2. 重启一个新的terminal

3. 启动示教器软件，将IP地址配置成192.168.1.100

4. 在terminal中切换到**/HandGuidingExample/release目录

5. 运行 ./Handguidingexample