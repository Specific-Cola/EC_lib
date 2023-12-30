# EC_lib
## 写在最前面
- 本代码框架旨在规范IRobot电控组代码使用
- 感谢 `Wanghongxi  &   湖南大学跃鹿战队`开源代码

### 结构
1. 算法库
2. 设备库
3. 板载库


#   ！！！！！！使用UTF-8！！！！！！

### 命名规范
#### 变量命名
1. 变量用小写 和 _ 组合，尽量单词写全，不要缩写
```
int my_command_interface;
```
2. 常量用全大写 和 _ 组合，
```
#define SPEED_KP       1000
#define SPEED_KI       100
#define SPEED_KD       100

const int MY_HARDWARE_INTERFACE = 1;
```



#### 函数命名
1. 采用驼峰命名法，与hal库函数区分开
```
void xiaoMiMotorSendMessage();
```


#### typedef命名
1.  首字母大写，_t为后缀
```
typedef struct{
    int x;
    int y;
    int z;
}Chassis_t;

```

### 各个bsp库之间应该解耦，尽量不要相互调用