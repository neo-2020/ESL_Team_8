/**
 * Authored by Diwakar
 * communication protocol - missing packets, checking crc etx
 * */

#include "in4073.h"
#include<string.h>

//read the packet
uint8_t read_packet()
{
    bool headercheck = false;
    do{
        pc_drone.head = dequeue(&rx_queue);
        headercheck = headerCheck(pc_drone.head);
    } while( !headercheck && (rx_queue.count >6));

    if(headercheck)
    {
        return mode;
    }
    pc_drone.pck_Type = dequeue(&rx_queue);
    pc_drone.roll = dequeue(&rx_queue);
    pc_drone.pitch = dequeue(&rx_queue);
    pc_drone.yaw = dequeue(&rx_queue);
    pc_drone.lift = dequeue(&rx_queue);

    char c1 = dequeue(&rx_queue);
    char c2 = dequeue(&rx_queue);
    pc_drone.crc = (uint16_t) ((c2 <<8) | c1);
    pckType_check();
    if(check_crc())
    {
        mode_set();
    }
    else if(nxt_packet())
    {
        mode_set();
    }
    else
    {
        if(rx_queue.count >7)
        {
            read_packet();
        }
    }
    return mode;

}

//check for header

bool headerCheck(uint8_t h)
{
    bool flag = false;
    char c = h;
    c = c >> 4;
    if(c == P_headerCheck)
    {
        flag = true;
    }
    return flag;
}

//broken packet restore
void restore_brokenPckt()
{
    brokenPck[0] = pc_drone.head;
    brokenPck[1] = pc_drone.pck_Type;
    brokenPck[2] = pc_drone.roll;
    brokenPck[3] = pc_drone.pitch;
    brokenPck[4] = pc_drone.yaw;
    brokenPck[5] = pc_drone.lift;
    brokenPck[6] = ((pc_drone.crc & 0xFF00 ) >> 8);
    brokenPck[7] = (pc_drone.crc && 0x00FF);
}

//header in broken packet
void header_brokenpckt()
{
    for (int i=0;i<8;i++)
    {
        if(headerCheck(brokenPck[i]))
        {
            for(int j=0;j<8;j++)
            {
                if((i+j) <8)
                {
                    brokenPck[j] = brokenPck[(j+i)];
                }
                else
                {
                    brokenPck[j] = dequeue(&rx_queue);
                }
            }
            break;
        }
    }
}

//broken packet check
bool brokenPckt_check()
{
    uint8_t * temp_pckt = brokenPck;
    uint16_t crc = compute_crc(temp_pckt, 6, NULL);
    uint16_t temp = ((brokenPck[6]<<8)| brokenPck[7]);

    return (temp == crc);
}

//packet type check
void pckType_check()
{
    switch(pc_drone.pck_Type)
    {
        case 10:
        if((int8_t) pc_drone.pitch < 40)
        {
            kp++;
        }
        else
        {
            kp--;
        }
        break;
        case 20:
        if((int8_t) pc_drone.pitch < 40)
        {
            kp1++;
        }
        else
        {
            kp1--;
        }
        break;
        case 30:
        if((int8_t) pc_drone.pitch <40)
        {
            kp2++;
        }
        else
        {
            kp2--;
        }
        break;
        case 40:
        if((int8_t) pc_drone.pitch <40)
        {
            kp1_pitch++;
        }
        else
        {
            kp1_pitch--;
        }
        break;
        case 50:
        if((int8_t) pc_drone.pitch <40)
        {
            kp2_pitch++;
        }
        else
        {
            kp2_pitch--;
        }
        break;
    }
}

//restoring it back to normal packets after broken packets
void restore_packet()
{
    pc_drone.head = brokenPck[0];
    pc_drone.pck_Type = brokenPck[1];
    pc_drone.roll = brokenPck[2];
    pc_drone.pitch = brokenPck[3];
    pc_drone.yaw = brokenPck[4];
    pc_drone.lift = brokenPck[5];
    pc_drone.crc = ((brokenPck[6] << 8) | brokenPck[7]);
}

//mode set
uint8_t mode_set()
{
    uint8_t recMode = (uint16_t) pc_drone.head & 0b00001111;

    if(recMode == 1)
    {
        panic =1;
        mode =1;
    }
    else
    {
        mode = recMode;
    }

    if(!mode_change_acknowledged)
    {
        if(recMode == mode)
        {
            mode_change_acknowledged = true;
        }
        else
        {
            send_mode_change();
        }
        
    }
    return recMode;
}

//next packet
bool nxt_packet()
{
    restore_brokenPckt();
    header_brokenpckt();
    if(brokenPckt_check())
    {
        restore_packet();
        return true;
    }
    else
    {
        return false;
    }
}

//crc check
bool check_crc()
{
    uint8_t check[6] = {pc_drone.head, pc_drone.pck_Type,
                        pc_drone.roll, pc_drone.pitch,
                        pc_drone.yaw, pc_drone.lift};
    uint8_t * temp = check;
    uint16_t crc = compute_crc(temp, 6, NULL);
    return (crc == pc_drone.crc);
}

//Sending packets from drone to pc

//set header
void set_Header()
{
    drone_pc.head = (uint8_t) P_Header | mode;
}

//kp packet
void kp_packet()
{
    drone_pc.dt1_1 = ((tx_queue.count & 0xFF00) >>8);
    drone_pc.dt1_2 = ((tx_queue.count & 0x00FF));
    drone_pc.dt2_1 = (uint8_t) timestamp;
    drone_pc.dt2_2 = (uint8_t) 0;
    drone_pc.dt3_1 = (uint8_t) (kp & 0x000000FF);
    drone_pc.dt3_2 = (uint8_t) (kp1 & 0x000000FF);
    drone_pc.dt4_1 = (uint8_t) (kp2 & 0x000000FF);
    drone_pc.dt4_2 = (uint8_t) 0;
}

//motor value packet
void motor_packet()
{
    drone_pc.dt1_1 = (uint8_t)((motor[0] & 0xFF00) >> 8);
    drone_pc.dt1_2 = (uint8_t)(motor[0] & 0x00FF);
    drone_pc.dt2_1 = (uint8_t)((motor[1] & 0xFF00) >> 8);
    drone_pc.dt2_2 = (uint8_t)(motor[1] & 0x00FF);
    drone_pc.dt3_1 = (uint8_t)((motor[2] & 0xFF00) >> 8);
    drone_pc.dt3_2 = (uint8_t)(motor[2] & 0x00FF);
    drone_pc.dt4_1 = (uint8_t)((motor[3] & 0xFF00) >> 8);
    drone_pc.dt4_2 = (uint8_t)(motor[3] & 0x00FF);
}

//mode change packet
void mode_change_packet()
{
    drone_pc.dt1_1 = (uint8_t)0;
    drone_pc.dt1_2 = (uint8_t)0;
    drone_pc.dt2_1 = (uint8_t)0;
    drone_pc.dt2_2 = (uint8_t)0;
    drone_pc.dt3_1 = (uint8_t)0;
    drone_pc.dt3_2 = (uint8_t)0;
    drone_pc.dt4_1 = (uint8_t)0;
    drone_pc.dt4_2 = (uint8_t)0;
}

//packet type set
void set_pckType(char temp)
{
    drone_pc.pckType = temp;

    switch (temp)
    {
    case 'm':
    motor_packet();
    if(mode == 5)
    {
        pckType = 'k';
    }
        break;
    case 'p':
    mode_change_packet();
    break;
    case 'o':
    break;
    case 'k':
    kp_packet()
    pckType = 'm';
    break;
    }
}

//packet on queue
void packet_on_queue()
{
    enqueue(&tx_queue, drone_pc.head);
    enqueue(&tx_queue, drone_pc.pckType);
    enqueue(&tx_queue, drone_pc.dt1_1);
    enqueue(&tx_queue, drone_pc.dt1_2);
    enqueue(&tx_queue, drone_pc.dt2_1);
    enqueue(&tx_queue, drone_pc.dt2_2);
    enqueue(&tx_queue, drone_pc.dt3_1);
    enqueue(&tx_queue, drone_pc.dt3_2);
    enqueue(&tx_queue, drone_pc.dt4_1);
    enqueue(&tx_queue, drone_pc.dt4_2);
}

//packet send
void send_packet(char type)
{
    // if(tx_queue.count != 0)
    // {
    //     memset(tx_queue, 0, 256);
    //     init_queue(tx_queue);
    // }

    set_Header();
    set_pckType(type);
    packet_on_queue();
    int i=0;
    while(i<10)
    {
        i = uart_put(i);
        timestamp++;
    }

}

//copy pasted from prev year lab code - acknowledging packet receive

void init_send_mode_change()
{
    mode_packet_change.head_p = '#';
    mode_packet_change.ender_p = '$';
    mode_change_acknowledged = true;
}

void set_acknowledge_flag(bool ack_flag)
{
    mode_change_acknowledged = ack_flag;
}

void send_mode_change()
{
    mode =0;
    bool ack = false;
    pckType = 'p';
    if(tx_queue.count != 0)
    {
        memset(tx_queue, 0, 256);
        init_queue(tx_queue);
    }
    if(tx_queue.count != 0)
    {
        memset(tx_queue, 0, 256);
        init_queue(tx_queue);
    }
    while(!ack)
    {
        mode =0;
        send_packet('p');
        if(rx_queue.count > 7)
        {
            if(prevAckMode != read_packet())
            {
                ack = true;
                //change mode
            }
        }
    }
    if(tx_queue.count != 0)
    {
        memset(tx_queue, 0, 256);
        init_queue(tx_queue);
    }
    if(tx_queue.count != 0)
    {
        memset(tx_queue, 0, 256);
        init_queue(tx_queue);
    }
}