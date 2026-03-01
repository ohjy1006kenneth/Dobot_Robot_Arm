#ifndef	__KSJ_CODE_H__
#define __KSJ_CODE_H__

// #pragma message("Include KSJCode.h") 


/*-------------------------------------------------
	Return Code
  ================================================*/
/** @brief 函数返回成功 */
#define		RET_SUCCESS			    0     
/** @brief 输入参数错误 */
#define		RET_PARAMERROR			-1     
/** @brief 内存分配错误 */
#define		RET_MALLOCFAIL			-2    
/** @brief 该功能不支持 */
#define		RET_NOTSUPPORT			-3     
/** @brief 相机不存在 */
#define		RET_DEVICENOTEXIST		-4     
/** @brief 相机没有初始化 */
#define		RET_DEVICENOTINIT		-5     
/** @brief 操作冲突 */
#define		RET_VIOLATION			-6     
/** @brief 权限错误 */
#define		RET_NOPRIVILEGE			-7     
/** @brief 失败（普通的失败，原因未知） */
#define		RET_FAIL			    -8     
/** @brief 失败（普通的失败，原因未知） */
#define		RET_WRONG			    -9     
/** @brief 相机恢复成功 */
#define     RET_RECOVERY_SUCCESS    -10    
/** @brief 相机恢复失败 */
#define     RET_RECOVERY_FAIL       -11    
/** @brief 帧错误 */
#define     RET_BADFRAME            -12    
/** @brief 无效帧 */
#define     RET_INVALIDFRAME        -13    
/** @brief 帧存相机会返回此值，表示采集图像数据0字节，错误的帧 */
#define     RET_ZEROFRAME           -14   
/** @brief 版本错误 */
#define     RET_VERSION_ERROR       -15    
/** @brief 当设置读取超时之后不进行恢复时，采集函数会返回此数值，而不会返回恢复的状态 */
#define     RET_TIMEOUT             -16   
/** @brief 相机已经关闭 */
#define     RET_DEVICECLOSED        -17	   
/** @brief 总线没有初始化 */
#define     RET_BUSNOTINIT          -18    
/** @brief 相机无法创建，通常因为缺少对应类型的相机库文件造成 */
#define     RET_CAM_NOT_CREATED     -19	   
/** @brief 非法的句柄 */
#define     RET_INVALID_HANDLE      -20	   
/** @brief 流模式没有打开 */
#define     RET_STREAMNOSTART       -21	   
/** @brief 相机3D功能没有初始化 */
#define     RET_3DNOTINIT           -22	   
/** @brief 相机不支持3D功能 */
#define     RET_3DNOTSUPPORT        -23	   
/** @brief 需要先停止流模式 */
#define     RET_STOPSTREAMFIRST     -24	   
/** @brief 数据IO错误，通常是因为数据传输有问题 */
#define     RET_DATAIO_ERROR        -25	   
/** @brief 相机3D功能初始化错误，可能因为相机不支持3D功能或者相机3D参数错误 */
#define     RET_3DINIT_ERROR        -26	  
/** @brief 无效相机 */
#define     RET_INVALID_DEVICE      -27	   
/** @brief 3D数据出现水印异常（在数据水印出现异常的时候，会提交一个3D帧数据） */
#define     RET_3DWATERMARK_ERROR   -28	   
/** @brief 当前3D帧未达到轮廓数，收到下一3D帧开始信号（收到新的3D开始信号，会将当前的3D数据提交一次） */
#define     RET_3DFRAME_RESTART     -29	   
/** @brief 正常完成一个3D帧（数据达到设定的轮廓数或者收到3D帧结束信号） */
#define     RET_3DFRAME_FULL        -30	   


#endif