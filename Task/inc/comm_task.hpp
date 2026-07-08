#ifndef COMM_TASK_HPP_
#define COMM_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void CommTaskInit(void);
void CommTask(void);

void UsbReceiveCallBack(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_TASK_HPP_ */