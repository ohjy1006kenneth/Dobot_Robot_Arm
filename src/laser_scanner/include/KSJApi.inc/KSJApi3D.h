#ifndef __KSJ_API_3D_H__
#define __KSJ_API_3D_H__

#include "KSJCode.h"

#if defined (_WIN32)
#  ifdef KSJAPI3D_EXPORTS
#    define KSJAPI3D_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define KSJAPI3D_IMPORT_EXPORT __declspec(dllimport)
#  endif

#  if defined (_M_IX86) || defined (__i386__)
#    define KSJAPI3D_CALLTYPE __stdcall
#  else
#    define KSJAPI3D_CALLTYPE  
#  endif

#elif defined (__GNUC__) && (__GNUC__ >= 4) && (defined (__linux__) || defined (__APPLE__))
#  define KSJAPI3D_IMPORT_EXPORT __attribute__((visibility("default")))
#  if defined (__i386__)
#    define KSJAPI3D_CALLTYPE __attribute__((stdcall))
#  else
#    define KSJAPI3D_CALLTYPE /* default */
#  endif
#  ifndef EXTERN_C
#    define EXTERN_C extern "C"
#  endif

#else
#  error Unknown platform, file needs adaption
#endif


#ifndef _WIN32
#define KSJ_API_3D  int
#define __stdcall
#else
#define KSJ_API_3D  KSJAPI3D_IMPORT_EXPORT int KSJAPI3D_CALLTYPE
#endif

#ifdef __cplusplus
extern "C" {
#endif
/** @defgroup Base 基本功能
 *  @{
 */
 
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Inital
	/// @brief     初始化KSJApi动态库
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 一般在程序初始化时调用
	///          \li 3D相机需在这个执行这个函数之前与电脑连接
	///          \li 可以调用多次，第二次调用如果发现已经初始化，那么将不进行任何操作，直接返回RET_SUCCESS
	///          \li 如果需要重新枚举3D相机，请执行反初始化KSJ3D_UnInit()后再执行KSJ3D_Inital()
	///          \li 特别需要注意的是：重新枚举操作以后，实际3D相机所对应的操作索引（库函数中的nIndex参数）可能会改变；所以不能通过操作索引（库函数中的nIndex参数）区别3D相机，可以通过函数KSJ3D_SetSerialNumber()给3D相机分配不同的序号（0-255，该序号保存到相机内部），通过序号区别3D相机。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Inital();

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_UnInitial
	/// @brief     反初始化KSJApi动态库
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_UnInitial函数反初始化后，API将释放所有已分配资源
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_UnInitial();
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetAPIVersion
	/// @brief     获取KSJAPI库的版本号
	/// @param     pnMaj1 [out] 返回主版本号1，一般做重要整体结构调整时会改变，增加1；主版本号1不同的API库不兼容
	/// @param     pnMaj2 [out] 返回主版本号2，一般做主要功能增加时会改变，增加1
	/// @param     pnMin1 [out] 返回次版本号1，一般做重要整体结构调整或者修改主要逻辑时会改变，增加1
	/// @param     pnMin2 [out] 返回次版本号2，一般在小的改动、修改Bug时会改变，增加1
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以在任何时候调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetAPIVersion(int *pnMaj1, int *pnMaj2, int *pnMin1, int *pnMin2);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDeviceCount
	/// @brief     得到所有凯视佳相机的数目（包括非3D相机）
	/// @param     pnCount [out] 返回相机的数目
	/// @return    连接到主机上的凯视佳相机的数目
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///          \li 如果相机已经连接电脑，无法得到正确的相机数目，可以尝试重新枚举相机（KSJ3D_UnInitial()，KSJ3D_Inital()）。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDeviceCount(int *pnCount);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDeviceInformation
	/// @brief     得到相机信息（型号，序号，硬件版本号）
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pusDeviceType [out]      返回相机型号的地址指针
	/// @param     pnSerialNumber [out]     返回相机序号的地址指针
	/// @param     pusFirmwareVersion [out] 返回相机固件版本号的地址指针
	/// @param     pusFpgaVersion [out]     返回相机FPGA版本号的地址指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///          \li 相机型号名称：可以通过函数KSJ3D_GetDeviceName将型号（pusDeviceType）转换为字符串。
	///          \li 相机序号：通过这个序号区别不同的实际相机，序号可以通过KSJ3D_SetSerialNumber()修改	
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDeviceInformation(int nIndex, int* pnDeviceType, int* pnSerialNumber, unsigned short *pwFirmwareVersion, unsigned short *pwFpgaVersion);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetSerialNumber
	/// @brief     设置3D相机序号
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     btSerials [in] 将序号设置到当前相机
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///          \li 序号即KSJ3D_GetDeviceInformation所获取的pnSerialNumber
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetSerialNumber(int nIndex, unsigned char btSerials);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetCameraName
	/// @brief     获得3D相机的名称
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     szCameraName [out] 返回相机字符串名称
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetCameraName(int nIndex, char szCameraName[64]);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_DeviceOpen
	/// @brief     打开指定索引的相机
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///          \li 执行KSJ3D_Inital之后，如果相机未被其他进程占用，默认打开该相机，无需再打开相机
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_DeviceOpen(int nIndex);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_DeviceClose
	/// @brief     关闭指定索引的相机
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///          \li 执行KSJ3D_Inital之后，如果相机未被其他进程占用，默认打开该相机；其他进程无法操作该相机，如果需要让其他进程操作该相机，需要调用该函数关掉该相机。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_DeviceClose(int nIndex);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_DeviceGetStatus
	/// @brief     获取相机在当前进程中的开启关闭状态
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pbOpen [out] 相机的开启状态
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_DeviceGetStatus(int nIndex, bool* pbOpen);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_LogSet
	/// @brief     设置Log日志是否输出及输出的保存目录
	/// @param     bEnable [in] 是否打开Log日志的输出功能
	/// @param     pszFolder [in] 输出的Log日志所保存的目录；可以为NULL或空字符串。
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以在任意时刻调用
	///				\li 如果pszFolder为NULL或空字符串，则Log输出目录会自动创建，目录位置在KSJAPI库所在目录下名称为KSJApiLog的目录
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_LogSet(bool bEnable, const char *pszFolder, int nLevel = 5);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SerialsDescReadout
	/// @brief     获取相机唯一序列号SN
	/// @param     nIndex [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     btDesc [out] 返回相机唯一序列号SN
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	/// @attention 唯一序列号SN即为相机的唯一号，与硬件标牌上SN一致，为8个宽字符数字
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SerialsDescReadout(int nIndex, unsigned char btDesc[16]);

/** @} */ 

/** @defgroup ROI ROI
 *  @{
 */
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetRoiMax
	/// @brief     获取3D ROI的范围
	/// @param     nIndex [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnColMax [out] 行宽的最大设置范围，单位：像素
	/// @param     pnRowMax [out] 行高的最大设置范围，单位：像素
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetRoiMax(int nIndex, int* pnColMax, int* pnRowMax);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetRoi
	/// @brief     设置视场范围
	/// @param     nIndex  [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nColStart [in] 列起始：ROI水平像素起始位置，以原始图像左上角为原点，这个值主要决定了X方向测量的起始位置。
	/// @param     nRowStart [in] 行起始：ROI垂直方向行起始位置，以原始图像左上角为原点，这个值主要决定了Z方向测量量程的最大值。
	/// @param     nColSize  [in] 列宽：ROI水平方向像素数，主要决定X方向的测量量程。
	/// @param     nRowSize  [in] 行高：ROI垂直方向像素数，主要决定Z方向的测量量程。
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetRoi(int nIndex, int nColStart, int nRowStart, int nColSize, int nRowSize);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetRoi
	/// @brief     获取视场范围
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnColStart [out] 列起始：ROI水平像素起始位置，以原始图像左上角为原点，这个值主要决定了X方向测量的起始位置。
	/// @param     pnRowStart [out] 行起始：ROI垂直方向行起始位置，以原始图像左上角为原点，这个值主要决定了Z方向测量量程的最大值。
	/// @param     pnColSize  [out] 列宽：ROI水平方向像素数，主要决定X方向的测量量程。
	/// @param     pnRowSize  [out] 行高：ROI垂直方向像素数，主要决定Z方向的测量量程。
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetRoi(int nIndex, int* pnColStart, int* pnRowStart, int* pnColSize, int* pnRowSize);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetMeasurementRangeMax
	/// @brief     获取相机的最大测量范围
	/// @param     nIndex    [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfZ       [out] Z方向的高度测量范围；单位：毫米
	/// @param     pfFovNear [out] X方向近端视野；单位：毫米
	/// @param     pfFovFar  [out] X方向远端视野；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 获得的是相机能够达到的最大测量范围
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetMeasurementRangeMax(int nIndex, float *pfZ, float *pfFovNear, float *pfFovFar);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetMeasurementRange
	/// @brief     获取相机当前的测量范围
	/// @param     nIndex    [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfZMin    [out] Z方向的高度相对零点最小测量位置；单位：毫米
	/// @param     pfZMax    [out] Z方向的高度相对零点最大测量位置；单位：毫米
	/// @param     pfFovNear [out] X方向近端视野；单位：毫米
	/// @param     pfFovFar  [out] X方向远端视野；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 获得的是当前相机ROI设置的测量范围
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetMeasurementRange(int nIndex, float *pfZMin, float *pfZMax, float *pfFovNear, float *pfFovFar);

/** @} */ 

/** @defgroup Param 参数
 *  @{
 */

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetExposureTimeRange
	/// @brief     获取相机的曝光设置范围
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfExpTimeMSMin [out] 最小曝光值，单位：毫秒
	/// @param     pfExpTimeMSMax [out] 最大曝光值，单位：毫秒
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetExposureTimeRange(int nIndex, float* pfExpTimeMSMin, float* pfExpTimeMSMax);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetExposureTime
	/// @brief     设置相机曝光值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     fExpTimeMS [in] 曝光值，单位：毫秒
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 传感器电子快门打开（感光元件开始进行感光）到快门关闭的时间。曝光值越大，感光时间越长，图像越亮，图像采集时间会变长，采集帧率会降低。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetExposureTime(int nIndex, float fExpTimeMS);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetExposureTime
	/// @brief     获取相机曝光值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     fExpTimeMS [out] 当前相机的曝光值，单位：毫秒
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetExposureTime(int nIndex, float* pfExpTimeMS);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetGainRange
	/// @brief     获取相机的增益范围
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnGainMin [out] 最小增益值
	/// @param     pnGainMax [out] 最大增益值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetGainRange(int nIndex, int* pnGainMin, int* pnGainMax);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetGain
	/// @brief     设置相机增益
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nGain [in] 增益值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 对感光信号进行信号放大，增益值越大，图像越亮，但同时也会对噪音信号进行放大，不宜设置过高。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetGain(int nIndex, int nGain);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetGain
	/// @brief     获取相机增益
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnGain [out] 当前相机的增益值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetGain(int nIndex, int* pnGain);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Set3DLaserLineBrightnessThreshold
	/// @brief     设置激光光条的灰度阈值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nThreshold [in] 激光光条的灰度阈值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 表示激光光条的灰度阈值，范围0到255。在原始图像上灰度值大于阈值的点被认为有效点参与点云计算，所以阈值越大，提取的点就会越少，适当地修改阈值可以减少噪声点对点云数据的干扰
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Set3DLaserLineBrightnessThreshold(int nIndex, int nThreshold);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Get3DLaserLineBrightnessThreshold
	/// @brief     获取激光光条的灰度阈值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnThreshold [in] 激光光条的灰度阈值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Get3DLaserLineBrightnessThreshold(int nIndex, int *pnThreshold);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Set3DLaserLineBrightnessLowThreshold
	/// @brief     设置激光光条的灰度低阈值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nLowThreshold [in] 激光光条的灰度阈值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 表示激光光条的灰度低阈值，范围0到255。激光光条的灰度低阈值小于正常阈值时有效
	///				\li 在计算光条中心时，首先会使用激光光条的灰度阈值进行判断，如果没有找到光条中心，那么会试图使用该低阈值再次进行计算
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Set3DLaserLineBrightnessLowThreshold(int nIndex, int nLowThreshold);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Get3DLaserLineBrightnessLowThreshold
	/// @brief     获取激光光条的灰度低阈值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnThreshold [in] 激光光条的灰度阈值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Get3DLaserLineBrightnessLowThreshold(int nIndex, int *pnLowThreshold);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Set3DLaserLineWidth
	/// @brief     设置激光光条的宽度值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nThreshold [in] 激光光条的宽度值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Set3DLaserLineWidth(int nIndex, int nWidth);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_Get3DLaserLineWidth
	/// @brief     获取激光光条的宽度值
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnThreshold [in] 激光光条的宽度值
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_Get3DLaserLineWidth(int nIndex, int *pnWidth);
/** @} */ 

/** @defgroup StartTrigger 起始结束触发
 *  @{
 */
	/** @brief 信号源枚举 */
	enum  	KSJ3D_START_TRIGGER_SOURCE {
		STS_INPUT_0 = 0,     /**< 开始采集信号源 INPUT0  */
	};

	/** @brief 触发条件 */
	enum  	KSJ3D_TRIGGER_EDGE_MODE {
		TEM_RISING_EDGE = 0,   /**< 上升沿  */
		TEM_FALLING_EDGE = 1,  /**< 下降沿  */
		TEM_HIGHLEVEL = 2,	   /**< 高电平有效  */
		TEM_LOWLEVEL = 3       /**< 低电平有效  */
	};
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetStartTrigger
	/// @brief     设置相机起始结束触发条件
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     StartTriggerSource [in] 信号输入接口，目前只有一个接口
	/// @param     bEnable [in] 是否启用起始结束触发
	/// @param     StartTriggerCondition [in] 起始结束触发条件，参考KSJ3D_TRIGGER_EDGE_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 设置传感器何时可以开始采集轮廓数据
	///				\li bEnable = true：采集时不需要“起始结束触发”条件，此时传感器只根据“轮廓触发”设置进行数据采集，采集到“采集轮廓数”指定的数值时，单次采集完成。
	///				\li 上升沿        ：指接收到起始结束传感器信号的上升沿后，才可以根据“轮廓触发”的设置开始采集轮廓数据，当采集到“采集轮廓数”指定的轮廓数时，单次采集完成。
	///				\li 下降沿        ：指接收到起始结束传感器信号的下降沿后，才可以根据“轮廓触发”的设置开始采集轮廓数据，当采集到“采集轮廓数”指定的轮廓数时，单次采集完成。
	///				\li 高电平        ：在起始结束触发信号的高电平期间进行轮廓采集，翻转到低电平时刻或提前达到“采集轮廓数”指定的轮廓数时，单次采集完成。
	///				\li 低电平        ：在起始结束触发信号的低电平期间进行轮廓采集，翻转到高电平时刻或提前达到“采集轮廓数”指定的轮廓数时，单次采集完成。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetStartTrigger(int nIndex, KSJ3D_START_TRIGGER_SOURCE  StartTriggerSource,  bool bEnable,  KSJ3D_TRIGGER_EDGE_MODE  StartTriggerCondition);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetStartTrigger
	/// @brief     获取相机起始结束触发条件
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pStartTriggerSource [out] 信号输入接口
	/// @param     pbEnable [out] 是否启用起始结束触发
	/// @param     pStartTriggerCondition [out] 触发条件，参考KSJ3D_TRIGGER_EDGE_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetStartTrigger(int nIndex, KSJ3D_START_TRIGGER_SOURCE *pStartTriggerSource, bool *pbEnable, KSJ3D_TRIGGER_EDGE_MODE *pStartTriggerCondition);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetStartTriggerParameters
	/// @brief     设置相机开始采集信号参数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nStartTriggerFilter [in] 信号滤波时间，单位：微秒
	/// @param     nStartTriggerDelay [in] 延迟脉冲，单位：个； 表示在收到开始信号后，将会忽略掉相应个数的脉冲才开始采集
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 延迟脉冲：指在起始结束触发信号有效后，延迟几个轮廓触发脉冲信号再开始轮廓触发采集
	///				\li 滤波：单位微秒，用于滤除起始结束外触发信号的高频干扰信号，当起始结束信号的高低电平持续时间在滤波时间范围内，则认为是无效信号
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetStartTriggerParameters(int nIndex, int nStartTriggerFilter, int nStartTriggerDelay);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetStartTriggerParameters
	/// @brief     获取相机开始采集信号参数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pStartTriggerFilter [out] 信号滤波时间，单位：微秒
	/// @param     pnStartTriggerDelay [out] 延迟脉冲，单位：个
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetStartTriggerParameters(int nIndex, int* pnStartTriggerFilter, int *pnStartTriggerDelay);

/** @} */ 


/** @defgroup DataTrigger 轮廓触发
 *  @{
 */
 	/** @brief 轮廓触发模式 */
	enum  	KSJ3D_DATA_TRIGGER_MODE
	{
		DTM_FREE_RUN = 0,            /**< 自由触发模式，传感器将自动以最大速度触发轮廓数据采集 */ 
		DTM_INTERNAL = 1,            /**< 内触发模式，传感器内部会根据“内触发频率”的设置触发轮廓采集 */ 
		DTM_EXTERNAL = 2             /**< 外部触发模式，传感器将根据外部编码器输入的触发信号、“分频”和“滤波”的设置进行轮廓采集 */ 
	};
	
 ///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetDataTriggerMode
	/// @brief     设置相机轮廓触发模式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     DataTriggerMode [in] 轮廓触发信号模式，参考KSJ3D_DATA_TRIGGER_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 设置传感器何时触发采集一个轮廓数据
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetDataTriggerMode(int nIndex, KSJ3D_DATA_TRIGGER_MODE DataTriggerMode);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDataTriggerMode
	/// @brief     获取相机轮廓触发模式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pDataTriggerMode [out] 轮廓触发信号模式，参考KSJ3D_DATA_TRIGGER_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDataTriggerMode(int nIndex, KSJ3D_DATA_TRIGGER_MODE *pDataTriggerMode);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetDataTriggerInternalFrequency
	/// @brief     设置内触发模式（DTM_INTERNAL）的触发频率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nInternalFrequencyHz [in] 触发频率
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li “内触发频率”需要根据实际的使用情况进行计算得到，计算之前我们需要知道如下信息：
	///				\li 1）物体Y方向运动速度，我们记为ν，单位：毫米/秒。
	///				\li 2）Y方向我们需要采集的轮廓间隔距离，我们记为Δy，单位：毫米。
	///				\li 然后通过这些信息，计算公式如下： 内触发频率f = ν/Δy，单位：Hz。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetDataTriggerInternalFrequency(int nIndex, int nInternalFrequencyHz);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDataTriggerInternalFrequency
	/// @brief     获取当前内部触发模式的触发频率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnInternalFrequencyHz [out] 触发频率
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDataTriggerInternalFrequency(int nIndex, int *pnInternalFrequencyHz);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetDataTriggerExternalTriggerParameters
	/// @brief     设置触发条件参数，只有轮廓触发模式为触发模式的时候有效
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nDataTriggerDivider  [in] 轮廓触发信号分频参数，表示接收多少个“轮廓触发”脉冲信号才触发一次轮廓数据采集
	/// @param     nDataTriggerDelay    [in] 轮廓触发延迟，当前无效
	/// @param     nDataTriggerFilter   [in] 轮廓触发信号滤波，单位：微秒；防止因为轮廓触发信号的噪声或边沿抖动而导致误触发，可以针对外部输入信号进行合适的滤波
	/// @param     DataTriggerCondition [in] 轮廓触发条件，参考KSJ3D_DATA_TRIGGER_MODE的定义，只能设置为上升沿或者下降沿
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetDataTriggerExternalTriggerParameters(int nIndex, int nDataTriggerDivider, int nDataTriggerDelay, int nDataTriggerFilter, KSJ3D_TRIGGER_EDGE_MODE DataTriggerCondition);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDataTriggerExternalTriggerParameters
	/// @brief     获取触发条件参数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnDataTriggerDivider  [out] 轮廓触发信号分频参数
	/// @param     pnDataTriggerDelay    [out] 轮廓触发延迟，当前无效
	/// @param     pnDataTriggerFilter   [out] 轮廓触发信号滤波，单位：微秒
	/// @param     pDataTriggerCondition [out] 轮廓触发条件，参考KSJ3D_DATA_TRIGGER_MODE的定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDataTriggerExternalTriggerParameters(int nIndex, int *pnDataTriggerDivider, int *pnDataTriggerDelay, int* pnDataTriggerFilter, KSJ3D_TRIGGER_EDGE_MODE *pDataTriggerCondition);
 
 	/// @brief设置线激光工作模式
	enum KSJ_LASER_MODE
	{
		LM_NORMAL_OPEN = 0,      ///< 常开：  激光一直亮
		LM_NORMAL_CLOSE = 1,     ///< 常关：  激光一直不亮
		LM_FLASH                 ///< 闪光灯：在传感器进行轮廓采集时激光亮，其他时候不亮
	};

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_LaserModeSet
	/// @brief     设置线激光工作模式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     LaserMode [in] 线激光工作模式， 参考KSJ_LASER_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_LaserModeSet(int nIndex, KSJ_LASER_MODE LaserMode);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_LaserModeGet
	/// @brief     获取线激光工作模式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pLaserMode [out] 线激光工作模式， 参考KSJ_LASER_MODE定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_LaserModeGet(int nIndex, KSJ_LASER_MODE *pLaserMode);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetYResolution
	/// @brief     设置运动方向精度
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     fYResolution [in] 运动方向精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 当“触发模式”设置为“Internal”时，计算之前我们需要知道如下信息：
	///				\li 1）物体Y方向运动速度，我们记为ν，单位：毫米/秒。
	///				\li 2）内触发频率，我们记为f，单位Hz。
	///				\li 然后通过这些信息，计算公式如下： 运动方向精度 = ν/f，单位：毫米。
	///				\li 当“触发模式”设置为“External”时，计算之前我们需要知道如下信息：
	///				\li 1）	编码器的精度，即编码器转一圈所输出的脉冲个数，我们记为n，单位：ppr。
	///				\li 2）	编码器转一圈物体移动距离，记为d，单位：毫米。
	///				\li 然后通过这些信息，计算公式如下： 运动方向精度 = d/n，单位：毫米。
	///				\li “运动方向精度”必须根据实际的情况进行填写，否则3D点云数据的Y方向数据会产生错误，导致3D点云发生变形。当设置的“运动方向精度”小于实际值，则3D点云数据会发生Y方向压缩，大于实际值，3D点云数据会发生Y方向拉伸

	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetYResolution(int nIndex, float fYResolution);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetYResolution
	/// @brief     获取运动方向精度
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfYResolution [out] 运动方向精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetYResolution(int nIndex, float *pfYResolution);


	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetUniformXResolution
	/// @brief     设置X方向重采样的分辨率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     fXResolution [in] X方向重采样分辨率精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用

	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetUniformXResolution(int nIndex, float fXResolution);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetUniformXResolution
	/// @brief     获取X方向重采样的分辨率参数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfXStart      [out] X方向重采样后起始位置；单位：毫米
	/// @param     pfXResolution [out] X方向重采样分辨率精度；单位：毫米
	/// @param     pnWidth       [out] X方向重采样后，一个轮廓的数据点数
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetUniformXResolution(int nIndex, float *pfXStart, float *pfXResolution, int* pnWidth);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetUniformXResolutionRange
	/// @brief     获取X方向重采样分辨率的范围
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfMin [out] X方向重采样分辨率的最小值；单位：毫米
	/// @param     pfMax [out] X方向重采样分辨率的最大值；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetUniformXResolutionRange(int nIndex, float *pfMin, float *pfMax);

 /** @} */ 

/** @defgroup Capture 采集
 *  @{
 */
	/// @brief     轮廓数据参数
	typedef struct _tag_PROFILE_DATA_PARAM
	{
		int     nPointNum;                  ///< 轮廓数据的数目
		float   fYMillimeters;              ///< 轮廓数据在运动方向相对于开始位置，单位：毫米
		int     nProfileIndex;              ///< 轮廓的序号，序号从0开始；表示实际处理的轮廓序号，不包括丢帧缺少的轮廓，该轮廓所对应的运动方向位置不能用这个序号计算
		int     nLostProfileNum;            ///< 该轮廓与上一个轮廓之间丢失的轮廓数
		unsigned long long  ullTimesTamp;   ///< 轮廓数据的时间戳
		unsigned char chStartSignalStatus;  ///< 该轮廓数据采集时刻，开始采集信号IO的电平状态，1为高电平，0位低电平
	}PROFILE_DATA_PARAM, *PPROFILE_DATA_PARAM;
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_LIVE_IMAGE_CALLBACK
	/// @brief     原始图像回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pData     [out] 指向当前图像数据的内存指针
	/// @param     nWidth    [out] 图像数据的宽度，单位：像素
	/// @param     nHeight   [out] 图像数据的高度，单位：像素
	/// @param     nBitCount [out] 图像数据的位图深度，单位：位
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterLiveImageCB时传入的上下文指针
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_LIVE_IMAGE_CALLBACK)(int nIndex, unsigned char *pData, int nWidth, int nHeight, int nBitCount, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterLiveImageCB
	/// @brief     设置原始图像的回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的原始图像回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterLiveImageCB(int nIndex, KSJ_LIVE_IMAGE_CALLBACK pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_PROFILE_DATA_CALLBACK
	/// @brief     轮廓数据回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nPointNum       [out] 当前轮廓数据的点数
	/// @param     fYMillimeters   [out] 当前轮廓数据在运动方向相对于开始轮廓的位置，单位：毫米
	/// @param     nProfileIndex   [out] 当前轮廓的序号，序号从0开始。
	/// @param     x               [out] 指向当前轮廓数据的x方向内存指针，单位：毫米
	/// @param     z               [out] 指向当前轮廓数据的z方向内存指针，单位：毫米
	/// @param     nLostProfileNum [out] 当前轮廓与上一个轮廓之间丢失的轮廓数
	/// @param     lpContext       [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterProfileDataCB时传入的上下文指针
	/// @attention 对于无效的数据点，x和z值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为x，z原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_PROFILE_DATA_CALLBACK)(int nIndex, int nPointNum, float fYMillimeters, int nProfileIndex, float *x, float *z, int nLostProfileNum, void *lpContext);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterProfileDataCB
	/// @brief     设置轮廓数据的回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的轮廓数据回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///				\li 仅在获取的数据格式为原始3D数据（KSJ3D_DF_NON_UNIFORM）时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterProfileDataCB(int nIndex, KSJ_PROFILE_DATA_CALLBACK pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_PROFILE_DATA_CALLBACKEX
	/// @brief     轮廓数据回调扩展函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     x         [out] 指向当前轮廓数据的x方向内存指针，单位：毫米
	/// @param     z         [out] 指向当前轮廓数据的z方向内存指针，单位：毫米
	/// @param     pParam    [out] 指向当前轮廓数据参数结构的内存指针，参数定义请参考PROFILE_DATA_PARAM
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterProfileDataCBEx时传入的上下文指针
	/// @attention 对于无效的数据点，x和z值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为x，z原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_PROFILE_DATA_CALLBACKEX)(int nIndex, float *x, float *z, PROFILE_DATA_PARAM* pParam, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetSystemClock
	/// @brief     获取系统时钟频率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfSysmClockMHz [out] 系统时钟频率，单位：MHz。1/pfSysmClockMHz为ullTimesTamp时间戳的时间单位，单位为us
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ_Init函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetSystemClock(int nIndex, float* pfSysmClockMHz);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterProfileDataCBEx
	/// @brief     设置轮廓数据回调扩展函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的轮廓数据回调扩展函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///				\li 仅在获取的数据格式为原始3D数据（KSJ3D_DF_NON_UNIFORM）时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterProfileDataCBEx(int nIndex, KSJ_PROFILE_DATA_CALLBACKEX pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_POINT_CLOUD_DATA_CALLBACK
	/// @brief     3D点云数据回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nTotalPointNum [out] 点云数据总点数：如果设置不丢掉无效数据，点云数据总点数=轮廓的数量X轮廓数据点数；如果设置丢掉数据，点云数据总点数<=轮廓的数量X轮廓数据点数
	/// @param     nProfileNum    [out] 轮廓的数量：表示实际采集到的轮廓数，不包括丢帧缺少的轮廓
	/// @param     x [out] 指向当前点云数据的x方向内存指针，单位：毫米
	/// @param     y [out] 指向当前点云数据的y方向内存指针，单位：毫米
	/// @param     z [out] 指向当前点云数据的z方向内存指针
	/// @param     nTotalLostProfileNum [out] 总共丢失的轮廓数
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterPointCloudDataCB时传入的上下文指针
	/// @attention 对于无效的数据点，x和z值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为x，z原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_POINT_CLOUD_DATA_CALLBACK)(int nIndex, int nTotalPointNum, int nProfileNum, float *x, float *y, float *z, int nTotalLostProfileNum, void *lpContext);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterPointCloudDataCB
	/// @brief     设置3D点云数据回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的3D点云数据回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///				\li 仅在获取的数据格式为原始3D数据（KSJ3D_DF_NON_UNIFORM）时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterPointCloudDataCB(int nIndex, KSJ_POINT_CLOUD_DATA_CALLBACK pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_UNIFORMX_PROFILE_DATA_CALLBACK
	/// @brief     X方向重排轮廓数据回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     z         [out] 指向当前轮廓数据的z方向内存指针
	/// @param     pParam    [out] 指向当前轮廓数据参数结构的内存指针，参数定义请参考PROFILE_DATA_PARAM
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterProfileDataCBEx时传入的上下文指针
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_UNIFORMX_PROFILE_DATA_CALLBACK)(int nIndex, float *z, PROFILE_DATA_PARAM* pParam, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterUniformXProfileDataCB
	/// @brief     设置X方向重排轮廓数据回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的X方向重排轮廓数据回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///				\li 仅在获取的数据格式为X方向数据重排的3D数据（KSJ3D_DF_UNIFORMX）时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterUniformXProfileDataCB(int nIndex, KSJ_UNIFORMX_PROFILE_DATA_CALLBACK pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_ZMAP_CALLBACK
	/// @brief     ZMap回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pData     [out] 指向轮廓图像数据的内存指针，像数值数据类型为浮点型
	/// @param     nWidth    [out] 图像数据的宽度，轮廓数据的点数，和设置X方向重排间距有关
	/// @param     nHeight   [out] 图像数据的高度，实际采集的轮廓数量
	/// @param     fYRes     [out] Y方向精度；单位：毫米
	/// @param     nTotalLostProfileNum [out] 总共丢失的轮廓数
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterZMapCB时传入的上下文指针
	/// @attention ZMap图像是点云数据的一种显示方式。以俯视物体的视角显示的一种灰度图，灰度图的宽是点云数据中一条轮廓的点数，灰度图的高是点云数据中包含的轮廓数，每一个像素的灰度对应了点云中一个点的高度。点云数据z值越大则灰度值越高，越小则灰度值越低。
	///				\li 对于无效的数据点，pData内的值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_ZMAP_CALLBACK)(int nIndex, float *pData, int nWidth, int nHeight, float fYRes, int nTotalLostProfileNum, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterZMapCB
	/// @brief     设置ZMap回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的ZMap回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 数据处理过程中，回调函数调用顺序为：原始图像回调=》轮廓数据回调=》3D点云数据/ZMap数据回调
	///				\li 仅在获取的数据格式为X方向数据重排的3D数据（KSJ3D_DF_UNIFORMX）时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_RegisterZMapCB(int nIndex, KSJ_ZMAP_CALLBACK pfCallback, void *lpContext);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_PI_CALLBACK
	/// @brief     PointCloud和Indensity回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nTotalPointNum [out] 点云数据总点数：如果设置不丢掉无效数据，点云数据总点数=轮廓的数量X轮廓数据点数；如果设置丢掉数据，点云数据总点数<=轮廓的数量X轮廓数据点数
	/// @param     nProfileNum    [out] 轮廓的数量：表示实际采集到的轮廓数，不包括丢帧缺少的轮廓
	/// @param     x [out] 指向当前点云数据的x方向内存指针，单位：毫米
	/// @param     y [out] 指向当前点云数据的y方向内存指针，单位：毫米
	/// @param     z [out] 指向当前点云数据的z方向内存指针，单位：毫米
	/// @param     idensity[out] 指向当前点云亮度数据内存指针
	/// @param     nTotalLostProfileNum [out] 总共丢失的轮廓数
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterPIDataCB时传入的上下文指针
	/// @attention 对于无效的数据点，x和z值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为x，z原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_PI_CALLBACK)(int nIndex, int nTotalPointNum, int nProfileNum, float *x, float *y, float *z, unsigned char *idensity, int nTotalLostProfileNum, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterPIDataCB
	/// @brief     设置PIData回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的PIData回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 仅在获取的数据格式KSJ3D_DF_NON_UNIFORM_INDENSITY时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D KSJ3D_RegisterPIDataCB(int nIndex, KSJ_PI_CALLBACK pfCallback, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ_ZI_CALLBACK
	/// @brief     ZMap和Indensity回调函数定义
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pData     [out] 指向轮廓图像数据的内存指针，像数值数据类型为浮点型
	/// @param     nWidth    [out] 图像数据的宽度，轮廓数据的点数，和设置X方向重排间距有关
	/// @param     nHeight   [out] 图像数据的高度，实际采集的轮廓数量
	/// @param     fYRes     [out] Y方向精度；单位：毫米
	/// @param     nTotalLostProfileNum [out] 总共丢失的轮廓数
	/// @param     idensity[out] 指向当前点云亮度数据内存指针
	/// @param     lpContext [out] 用户上下文指针，这个指针是用户调用KSJ3D_RegisterZIDataCB时传入的上下文指针
	/// @attention ZMap和亮度图一起返回的回调函数
	///				\li 对于无效的数据点，z内的值被赋值为-1000；可以通过该值排除无效的数据点
	///				\li 全视野的左下角为原点位置
	///
	///-----------------------------------------------------------------------------
	typedef void(__stdcall *KSJ_ZI_CALLBACK)(int nIndex, float *z, int nWidth, int nHeight, float fYRes, int nTotalLostProfileNum, unsigned char* idensity, void *lpContext);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_RegisterZIDataCB
	/// @brief     设置ZIData回调函数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfCallback [in] 用户定义的ZMap回调函数指针
	/// @param     lpContext  [in] 回调函数的上下文指针
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 仅在获取的数据格式为KSJ3D_DF_UNIFORMX_INDENSITY时，该回调才会被调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D KSJ3D_RegisterZIDataCB(int nIndex, KSJ_ZI_CALLBACK pfCallback, void *lpContext);

	/** @brief 用户预设组定义 */
	typedef enum
	{
		KSJ3D_DF_NON_UNIFORM = 0,           /**< 原始3D数据 */
		KSJ3D_DF_UNIFORMX = 1,     /**< X方向数据重排的3D数据 */
		KSJ3D_DF_NON_UNIFORM_INDENSITY = 2,  /**< 原始3D数据和亮度图*/
		KSJ3D_DF_UNIFORMX_INDENSITY = 3,  /**< X方向数据重排的3D数据和亮度图*/
	}KSJ3D_DATA_FORMAT;

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetDataFormat
	/// @brief     设置3D图像数据格式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     DataFormat [in] 3D图像数据格式，参考KSJ3D_DATA_FORMAT类型定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetDataFormat(int nIndex, KSJ3D_DATA_FORMAT DataFormat);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetDataFormat
	/// @brief     获取3D图像数据格式
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pDataFormat [out] 当前的3D图像数据格式，参考KSJ3D_DATA_FORMAT类型定义
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetDataFormat(int nIndex, KSJ3D_DATA_FORMAT* pDataFormat);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetMaxNumberOfProfilesToCapture
	/// @brief     设置采集一次ZMap或3D点云数据最多实际采集的轮廓数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nNumberOfProfiles [in] 采集轮廓数
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 这个值需要根据实际的使用情况进行计算得到，计算之前我们需要知道如下信息：
	///				\li 1）物体Y方向的尺寸，我们记为y，单位：毫米。
	///				\li 2）Y方向实际采集的轮廓间隔距离，我们记为Δy，单位：毫米。
	///				\li 然后通过这些信息，计算公式如下：采集轮廓数N=y/Δy，单位：个。
	///				\li 设置采集轮廓数设置越大，采集一次ZMap或3D点云数据的时间越长。
	///				\li 特别需要注意：实际采集的轮廓数有可能与“采集轮廓数”的设置不同，该轮廓数表示实际采集到的数据，不包括丢帧丢失的轮廓数据，具体请参考相机说明书
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_SetMaxNumberOfProfilesToCapture(int nIndex, int nNumberOfProfiles);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetMaxNumberOfProfilesToCapture
	/// @brief     获取采集一次ZMap或3D点云数据需要最多采集的轮廓数
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pNumberOfProfiles [out] 采集轮廓数
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetMaxNumberOfProfilesToCapture(int nIndex, int* pNumberOfProfiles);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_StartAcquisition
	/// @brief     启动相机开始采集工作
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 相机开始采集工作之后，相机的一些参数将无法改动，比如：ROI，激光光线阈值等
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_StartAcquisition(int nIndex);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_StopAcquisition
	/// @brief     停止相机采集工作
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_StopAcquisition(int nIndex);


	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetObliterateInvalidData
	/// @brief     设置是否自动去掉无效的数据点
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     bObliterate [in] 是否自动删除无效的点；如果为true，自动删除掉无效的数据点，如果为false，保留无效的数据点
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 如果保留无效的数据点，无效数据点的高度值（Z）被设定为-1000
	///				\li 如果自动去掉无效的数据点，因为无效数据点被删除掉，那么轮廓数据和3D点云数据的数据数目是变化的。
	///				\li 仅在获取的数据格式为原始3D数据（KSJ3D_DF_NON_UNIFORM）时，该参数才起作用，否者不支持
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_SetObliterateInvalidData(int nIndex, bool bObliterate);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetObliterateInvalidData
	/// @brief     获取数据回调是否去掉无效的点
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     bObliterate [out] 是否自动删除无效的点
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 仅在获取的数据格式为原始3D数据（KSJ3D_DF_NON_UNIFORM）时，该参数才起作用，否者不支持
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_GetObliterateInvalidData(int nIndex, bool* bObliterate);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetImageBufferNumber
	/// @brief     设置API图像缓存数目
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     nNumber [in] 图像帧缓存数目
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li API采集工作时，会从传感器采集图像放入缓存中，缓存数目越大，占用更多的内存，越能够防止由于PC性能波动造成的丢帧。
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_SetImageBufferNumber(int nIndex, int nNumber);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetImageBufferNumber
	/// @brief     获取API图像缓存数目
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pnNumber [out] 图像缓存数目
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_GetImageBufferNumber(int nIndex, int* pnNumber);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_SetCaptureTimeout
	/// @brief     设置采集超时时间
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     dwTimeOut [in] 采集超时时间,单位为毫秒
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用，外触发时，这个时间最好大于外触发可能的最大时间间隔
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_SetCaptureTimeout(int nIndex, unsigned int dwTimeOut);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetCaptureTimeout
	/// @brief     获取采集超时时间
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pdwTimeOut [in] 返回的当前超时时间值,单位为毫秒
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非0值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D   KSJ3D_GetCaptureTimeout(int nIndex, unsigned int* pdwTimeOut);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetTransmissionRate
	/// @brief     获取采集帧率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfTransmissionRate [out] 采集帧率
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 这个值代表的是当前实际采集过程中轮廓数据的采集频率
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetTransmissionRate(int nIndex, float *pfTransmissionRate);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetExternalTriggerRate
	/// @brief     获取轮廓触发频率
	/// @param     nIndex [in] 相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfExternalTriggerRateHz [out] 轮廓触发频率
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 当“轮廓触发”的“触发模式”设置为“Internal”时，是传感器内部根据“内触发频率”的设置实际产生的触发频率。
	///				\li 当“轮廓触发”的“触发模式”设置为“External”时，是滤波后、分频前的轮廓外触发信号的输入频率。

	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetExternalTriggerRate(int nIndex, float *pfExternalTriggerRateHz);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_GetZMapUniformXYSize
	/// @brief     3D点云数据X,Y方向等距重采样数据大小
	/// @param     nOutWidth  [out]  重采样之后3D数据横向(X方向)数据点数
	/// @param     nOutHeight [out]  重采样之后3D数据纵向(Y方向)数据点数
	/// @param     nWidth     [in]   3D数据横向宽度(X方向)
	/// @param     nHeight    [in]   3D数据纵向高度(Y方向)
	/// @param     fXRes      [in]   X方向精度；单位：毫米
	/// @param     fYRes      [in]   Y方向精度；单位：毫米
	/// @param     fRes       [in]   重采样精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以任意时刻调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_GetZMapUniformXYSize(int *nOutWidth, int *nOutHeight, int nWidth, int nHeight, float fXRes, float fYRes, float fRes);
	
	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_ZMapUniformXY
	/// @brief     ZMap数据X,Y方向等距重采样。解决ZMap数据X与Y方向的精度不同导致的变形问题
	/// @param     nIndex  [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfZOut  [out] 重采样之后存放3D数据的内存地址指针
	/// @param     pfZIn   [in]  原始3D数据的内存地址指针
	/// @param     nWidth  [in]  3D数据横向宽度(X方向)数据点数
	/// @param     nHeight [in]  3D数据纵向高度(Y方向)数据点数
	/// @param     fXRes   [in]  X方向精度；单位：毫米
	/// @param     fYRes   [in]  Y方向精度；单位：毫米
	/// @param     fRes    [in]  重采样精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 存放3D数据的内存必须提前分配，需要的数据内存大小，由函数KSJ3D_GetZMapUniformXYSize计算
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_ZMapUniformXY(int nIndex, float* pfZOut, float* pfZIn, int nWidth, int nHeight, float fXRes, float fYRes, float fRes);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_ZMapIndensityUniformXY
	/// @brief     ZMap与对应的亮度数据X,Y方向等距重采样。解决ZMap数据X与Y方向的精度不同导致的变形问题
	/// @param     nIndex  [in]  相机索引（从0开始，最大索引数为:连接到主机的相机数目减一）
	/// @param     pfZOut  [out] 重采样之后存放3D数据的内存地址指针
	/// @param     pfZIn   [in]  原始3D数据的内存地址指针
	/// @param     pIOut   [out] 重采样之后存放亮度数据的内存地址指针
	/// @param     pIIn    [in]  原始亮度数据的内存地址指针
	/// @param     nWidth  [in]  重彩样前数据横向宽度(X方向)数据点数
	/// @param     nHeight [in]  重彩样前数据纵向高度(Y方向)数据点数
	/// @param     fXRes   [in]  X方向精度；单位：毫米
	/// @param     fYRes   [in]  Y方向精度；单位：毫米
	/// @param     fRes    [in]  重采样精度；单位：毫米
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 调用KSJ3D_Inital函数初始化后调用
	///				\li 存放3D数据的内存必须提前分配，需要的数据内存大小，由函数KSJ3D_GetZMapUniformXYSize计算
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D KSJ3D_ZMapIndensityUniformXY(int nIndex, float* pfZOut, float* pfZIn, unsigned char* pIOut, unsigned char* pIIn, int nWidth, int nHeight, float fXRes, float fYRes, float fRes);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_HelperSaveToPCD
	/// @brief     将3D点云数据保存为PCD数据文件
	/// @param     nWidth [in] 3D数据帧的宽度
	/// @param     nHeight [in] 3D数据帧的高度，轮廓数目
	/// @param     pfX [in] 保存3D数据帧的X方向的数据地址指针
	/// @param     pfY [in] 保存3D数据帧的Y方向的数据地址指针
	/// @param     pfZ [in] 保存3D数据帧的Z方向的数据地址指针
	/// @param     pszFileName [in] PCD文件路径
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以任意时刻调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_HelperSaveToPCD(int nWidth, int nHeight, float *pfX, float *pfY, float *pfZ, const char *pszFileName);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_HelperSaveToPCDEx
	/// @brief     将3D点云数据保存为PCD数据文件,同时把亮度数据加到点云中
	/// @param     nWidth [in] 3D数据帧的宽度
	/// @param     nHeight [in] 3D数据帧的高度，轮廓数目
	/// @param     pfX [in] 保存3D数据帧的X方向的数据地址指针
	/// @param     pfY [in] 保存3D数据帧的Y方向的数据地址指针
	/// @param     pfZ [in] 保存3D数据帧的Z方向的数据地址指针
	/// @param     pIndensity [in] 保存3D数据帧的亮度数据地址指针
	/// @param     pszFileName [in] PCD文件路径
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以任意时刻调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_HelperSaveToPCDEx(int nWidth, int nHeight, float *pfX, float *pfY, float *pfZ, unsigned char* pIndensity, const char *pszFileName);

	///-----------------------------------------------------------------------------
	///
	/// @brief     KSJ3D_HelperSaveToTiff
	/// @brief     将重采样后的3D点云数据保存为TIFF数据文件
	/// @param     nWidth [in] 3D数据帧的宽度
	/// @param     nHeight [in] 3D数据帧的高度，轮廓数目
	/// @param     pfZ [in] 保存3D数据帧的Z方向的数据地址指针
	/// @param     pszFileName [in] TIFF文件路径
	/// @return    成功返回 RET_SUCCESS(1)。否则返回非1值的错误码, 请参考 KSJCode.h 中错误码的定义。
	/// @attention 可以任意时刻调用
	///
	///-----------------------------------------------------------------------------
	KSJ_API_3D  KSJ3D_HelperSaveToTiff(int nWidth, int nHeight, float* pfZ, const char* pszFileName);

 /** @} */ 

#ifdef __cplusplus
}
#endif

#endif
