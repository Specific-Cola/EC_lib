# EC_lib
### �ṹ
1. �㷨��
2. �豸��
3. ���ؿ�

### �����淶
#### ��������
1. ������Сд �� _ ��ϣ���������дȫ����Ҫ��д
```
int my_command_interface;
```
2. ������ȫ��д �� _ ��ϣ�
```
#define SPEED_KP       1000
#define SPEED_KI       100
#define SPEED_KD       100

const int MY_HARDWARE_INTERFACE = 1;
```



#### ��������
1. �����շ�����������hal�⺯�����ֿ�
```
void xiaoMiMotorSendMessage();
```


#### typedef����
1.  ����ĸ��д��_tΪ��׺
```
typedef struct{
    int x;
    int y;
    int z;
}Chassis_t;

```