#include "mbed.h"

CAN can1(PA_11, PA_12, 1000000); // CANの初期化 (例: PA_11:RX, PA_12:TX, 1000kbps)
CAN can2(PB_12, PB_13, 1000000);
//MDD1のパラメータデータ設定
uint8_t data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data2[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data3[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data4[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data5[4] = {0x00, 0x00, 0x00, 0x00};
//MDD1の目標値データ設定
uint8_t data6[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//サーボの目標値設定
uint8_t data7[6] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//MDD1のパラメータCAN設定
CANMessage msg1_tx{0x200, data1, sizeof(data1)};
CANMessage msg2_tx{0x201, data2, sizeof(data2)};
CANMessage msg3_tx{0x202, data3, sizeof(data3)};
CANMessage msg4_tx{0x203, data4, sizeof(data4)};
CANMessage msg5_tx{0x210, data5, sizeof(data5)};
//MDD1の目標値CAN設定
//受信データの受け入れ先
CANMessage msg_rx1;
CANMessage msg_rx2;
uint8_t can1_data_rx1[8];
uint8_t can1_data_rx2[8];
uint8_t can1_data_rx3[8];
uint8_t can2_data_rx1[8];
uint8_t can2_data_rx2[8];
uint8_t can2_data_rx3[8];
int x_data = 0,y_data = 0;
//サーボの目標値CAN設定
CANMessage servo_tx{0x300, data7, sizeof(data7)};
void can_tx(int a){
    switch (a) {
        case 0:
            can1.write(msg1_tx);
            break;
        case 1:
            can1.write(msg2_tx);
            break;
        case 2:
            can1.write(msg3_tx);
            break;
        case 3:
            can1.write(msg4_tx);
            break;
        case 4:
            can1.write(msg5_tx);
            break;
        case 5:
            can1.write(servo_tx);
        default:
            break;
    }
    printf(" count:%d", a);
}
void can_rx(){
    while(can1.read(msg_rx1)){
        switch((msg_rx1.id)){
            case 0x230:
                for(int i=0;i<8;i++)can1_data_rx1[i] = msg_rx1.data[i];
                //printf(" RX ID:0x%03X Data2:", msg_rx1.id);
                //for (int i = 0; i < msg_rx1.len; i++)printf("%02X", msg_rx1.data[i]);
            default:
                for(int i=0;i<8;i++)can1_data_rx2[i] = msg_rx1.data[i];
                //printf(" RX ID:0x%03X Data3:", msg_rx1.id);
                //for (int i = 0; i < msg_rx1.len; i++)printf("%02X", msg_rx1.data[i]);
        }
    }
    while(can2.read(msg_rx2)){
        switch((msg_rx2.id)){
            case 0x100:
                for(int i=0;i<8;i++)can2_data_rx1[i] = msg_rx2.data[i];
                printf(" RX ID:0x%03X Data1:", msg_rx2.id);
                for (int i = 0; i < msg_rx2.len; i++)printf("%02X", msg_rx2.data[i]);
        }
    }
}
void yamasiro_arm(int x, int y){
    int16_t out_motor[4];
    int mtr_x[2] = {0,0},mtr_y[2] = {0,0};
    if(x<0){
        mtr_x[0] = -8;
        mtr_x[1] = -8;
    }
    else if(x>0){
        mtr_x[0] = 8;
        mtr_x[1] = 8;
    }
    else if (x == 0){
        mtr_x[0] = 0;
        mtr_x[1] = 0;
    }
    if(y<0){
        mtr_y[0] = 7;
        mtr_y[1] = -7;
    }
    else if(y>0){
        mtr_y[0] = -7;
        mtr_y[1] = 7;
    }
    else if(y == 0){
        mtr_y[0] = 0;
        mtr_y[1] = 0;
    }
    out_motor[0] = mtr_x[0] + mtr_y[0];
    out_motor[1] = mtr_x[1] + mtr_y[1];
    data6[2] = (out_motor[0] >> 4);
	data6[3] = out_motor[0];
    data6[4] = (out_motor[1] >> 4);
	data6[5] = out_motor[1];
    printf("mtr1:%03X ", out_motor[0]);
    printf("mtr2:%03X\n", out_motor[1]);
}
int main() {
    int count = 0;
    while (true) {
        CANMessage msg6_tx{0x220, data6, sizeof(data6)};
        can_rx();
        if(can2_data_rx1[0] == 1)x_data = 1;
        else if(can2_data_rx1[1] == 1)x_data = -1;
        else x_data = 0;
        if(can2_data_rx1[2] == 1)y_data = 1;
        else if(can2_data_rx1[3] == 1)y_data = -1;
        else y_data = 0;
        yamasiro_arm(x_data,y_data);
        if (can1.read(msg_rx1)) {
            if (can1_data_rx1[5] == 0) {
                printf(" Sending params and mode");
                can_tx(count);
                count++;
            } else if (can1_data_rx1[5] == 1) {
                printf(" Sending target");
                can1.write(msg6_tx);
                for(int a=0;a<6;a++)printf("%08X ", data6[a]);
            }
        }
        if(count > 6)count = 0;
    }
}