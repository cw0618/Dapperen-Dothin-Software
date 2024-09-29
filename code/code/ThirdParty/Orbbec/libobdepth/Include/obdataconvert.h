#ifndef __MODULE_OBDEPTH_H_
#define __MODULE_OBDEPTH_H_
#include <stdint.h>
#include "obdepthtypes.h"

#ifdef __cplusplus
extern "C"{
#endif


    typedef enum ProjectID{
		OB_PROJECT_DISP12_RAW12_UNPACK =100,  //!<视差12bit  mipi raw12传输，平台自动解包
		OB_PROJECT_DISP12_RAW10_UNPACK = 101,			//!<视差12bit  mipi raw10传输，平台自动解包
		OB_PROJECT_DISP12_RAW10_PACK = 102,			//!<视差12bit  mipi raw10传输，需要解包
		OB_PROJECT_DISP12_RAW12_PACK,	//!<视差12bit  mipi raw12传输，需要解包
		OB_PROJECT_DISP10_RAW10_UNPACK = 104,			//!<视差10bit  mipi raw10传输，平台自动解包
		OB_PROJECT_DISP10_RAW10_PACK = 105,				//!<视差10bit  mipi raw10传输，需要解包 

		OB_PROJECT_DISP12_RAW12_UNPACK_NOFILTER = 106,  //!<视差12bit  mipi raw12传输，平台自动解包
		OB_PROJECT_DISP12_RAW10_UNPACK_NOFILTER,			//!<视差12bit  mipi raw10传输，平台自动解包
		OB_PROJECT_DISP12_RAW10_PACK_NOFILTER,			//!<视差12bit  mipi raw10传输，需要解包
		OB_PROJECT_DISP12_RAW12_PACK_NOFILTER,				//!<视差12bit  mipi raw12传输，需要解包
		OB_PROJECT_DISP10_RAW10_UNPACK_NOFILTER,			//!<视差10bit  mipi raw10传输，平台自动解包
		OB_PROJECT_DISP10_RAW10_PACK_NOFILTER,				//!<视差10bit  mipi raw10传输，需要解包 
		
    }obc_project_id_t;


	typedef enum SoftFilterType {
		OB_SOFTFILTER_TYPE_NONE = 0,			//!< 不使用滤波
		OB_SOFTFILTER_TYPE_ARM ,				//!< 使用ARM滤波
		OB_SOFTFILTER_TYPE_DSP, 					//!< 使用DSP滤波
		OB_SOFTFILTER_TYPE_END 					
	}obc_softfilter_type_t;

	//nPreciseUnit
	typedef enum PreciseUnit_ {
		OB_DEPTH_PRECISE_UNIT_0_1MM = 1,			//!< 单位，0.1mm
		OB_DEPTH_PRECISE_UNIT_1MM=10			//!< 单位，1mm
	}obc_depth_precise_unit_t;


    typedef struct base_s2dconfig{

        float fx;
        /** The zero plane pixel size 标定平面的一个量化数值 */
        float baseline;
        /** The zero plane distance in depth units.标定距离 */
        float z0;
        /*
        * input data packed fmt: MIPI, 1; unpack, 0
        */
        uint8_t mipi_packed;

        /*传输时的数据类型:@see mipi_transfer_datatype  10: raw10   12 :raw 12  100:12bit as raw10*/
	    uint8_t ntransfer_datatype;

        //最小深度限制
	    uint32_t ndepth_mincutoff;
        //最大深度限制
        uint32_t ndepth_maxcutoff;

        uint8_t  enablefilter;
    } ob_base_s2dconfig_t;


    /**
     * @brief 获取深度库版
     * 
     * @param  uint32_t * versioncode  as [MAJOR,MINOR,PATCH,RELEASER]
     *
     * @return OBC_API_EXPORT int
     */
	OBC_API_EXPORT int obc_getversion(uint32_t * versioncode);


      /**
     * @brief 获取深度库版
     * 
     * @param  uint32_t * versioncode  as [MAJOR,MINOR,PATCH,RELEASER]
     *
     * @return OBC_API_EXPORT int
     */
	OBC_API_EXPORT int obc_get_dspsoftfilter_version(uint32_t * versioncode);

    /**
     * @brief 获取软件滤波库版
     typedef struct
     {
     uint8_t major;
     uint8_t minor;
     uint8_t maintenance;
     uint8_t build;
     } ;
     * 
     * @param  uint32_t * versioncode
     *
     * @return OBC_API_EXPORT int
     */
	 OBC_API_EXPORT int obc_get_softfilter_version(uint32_t * versioncode);

    /**
     * Init Args by the config struct
     * @param config  The complete path to the configuration file
     * @param cfgsize  the pconfig size
     * @return  0 success, < 0 failed
     */
    OBC_API_EXPORT int obc_init(const s2dconfig_t* pconfig, int cfgsize, const void* obcops);
    /**
     * Init Args by the config struct
     * @param config  The complete path to the configuration file
     * @param cfgsize  the pconfig size
     * @return  0 success, < 0 failed
     */
    OBC_API_EXPORT int obc_init2(const ob_base_s2dconfig_t* pconfig, int cfgsize, const void* obcops);

    /**
     * @brief  obc_release
     * 
     * @return  void
     */
    OBC_API_EXPORT void obc_release();

	/**
	* @brief  dump s2dtable
	* @param  table_filename  
	* @return  0: success; <oth>:error
	*/
	OBC_API_EXPORT int obc_dump_s2dtable(const char * table_filename);

	/**
	* @brief   obc_reload_ref  when  ref file change
	* @param  refBuf
	* @param  len
	* @return  0: success; <oth>:error
	*/
	OBC_API_EXPORT int obc_reload_ref(const unsigned char * refBuf, int len);

    /**
     * @brief  Init Args by the ref file
     * 
     * @param  const unsigned char * refBuf  参考图  存放与内存中
     * @param  int size    参考图大小
     * @param  const void * obcops
     *
     * @return OBC_API_EXPORT int
     */
    OBC_API_EXPORT int obc_init_by_ref(const unsigned char *refBuf, int size, const void* obcops);


	/**
	* @brief  Init Args by the ref file
	*
	* @param  project_id
	* @param  const unsigned char * refBuf  参考图  存放与内存中
	* @param  int size    参考图大小
	* @param  const void * obcops
	*
	* @return OBC_API_EXPORT int
	*/
	OBC_API_EXPORT int obc_init_by_project_id_and_ref(int project_id, const unsigned char *refBuf, int size, const void* obcops);

    
    /**
     * @brief  Init Args by project id 
     *
     * @param  project_id
     * @param  const unsigned char * refBuf  参考图  存放与内存中
     * @param  int size    参考图大小
     * @param  const void * obcops
     *
     * @return OBC_API_EXPORT int
     */
    OBC_API_EXPORT int obc_init_by_project_id(int project_id, float baseline,float fx,float z0, const void* obcops);
    /**
     * @brief  Init Args by default in the code
     * 
     * @param  const void * obcops
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int  obc_init_by_default_param(const void* obcops);


   

    /**
     * convert 16bit shift data to depth data
     * @param [in] src 16bit unpack raw data
     * @param [out] dst 16bit depth data
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_s2d(const uint16_t *src, uint16_t *dst, int w, int h);

    /**
     * get current frame if it's exist datatype
     * @return   see  define in obdepth_stream_type_t
     */
    OBC_API_EXPORT int obc_get_datatype();

	/**
	* obc_set_depth_cutoff
	* @param [in]   min   unit 0.1mm
	* @param [in]   max   unit 0.1mm
	* @return   0:success  ;<oth> :error
	*/
	OBC_API_EXPORT int  obc_set_depth_cutoff(uint32_t min, uint32_t max);


	/**
	* obc_set_baseline   z0 fx
	* @param [in]   baseline   
	* @param [in]   z0   
	* @param [in]   fx
	* @return   0:success  ;<oth> :error
	*/
	OBC_API_EXPORT int  obc_set_baseline(float baseline, float z0, float fx);

	/**
	* obc_get_preciseunit 
	* @param [out] precise_unit see  define in obc_precise_unit_t
	* @return  0：success；<oth>: ERROR 
	*/
	OBC_API_EXPORT int obc_get_preciseunit(int *precise_unit);


	/**
	* obc_set_preciseunit
	* @param [int] precise_unit see  define in obc_precise_unit_t
	* @return  0：success；<oth>: ERROR
	*/
	OBC_API_EXPORT int obc_set_preciseunit(int precise_unit);

    /**
     * rotation the image
     * @param [in] src 16bit 
     * @param [out] dst 16bit 
     * @param [in] w   width
     * @param [in] h   height
     * @param [in] degree   support -90,0,90
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_rotation(const uint16_t *src, uint16_t* dst, int w, int h, int degress);

    /**obc_enable_origin
     * @param [in] enable
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_enable_origin(int enable);

   


	/**
	* @brief  get soft filter status 
	*
	* @param  [out] en  1: open ; 0:close
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int  obc_get_softfilter_status(uint8_t *en);


	/**
	* @brief  set soft filter status
	*
	* @param  [in] en  1: open ; 0:close
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int  obc_set_softfilter_status(uint8_t en);
	/**
	* @brief  set soft filter type
	*
	* @param  filter_type  @see obc_softfilter_type_t
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int obc_set_post_filter_type(int filter_type);

	/**
	* @brief  get soft filter type
	*
	* @param  [out]filter_type  @see obc_softfilter_type_t
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int obc_get_post_filter_type(int * filter_type);

	/**
	* @brief  obc_get_transfer_type
	*
	* @param  [out]filter_type  @see obc_transfer_datatype_t
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int obc_get_transfer_type(int * transfer_type);


	/**
	* @brief  obc_set_transfer_type
	*
	* @param  [out]filter_type  @see obc_transfer_datatype_t
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int obc_set_transfer_type(int  transfer_type);


    /**
     * UnPack  Raw data.
     * @param [in] src  raw data
     * @param [out] dst 16bit unpack data, High four place fill 0
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_raw_2_ir(const uint8_t *src, uint16_t *dst, int w, int h);

    /**
     * UnPack  Raw data.
     * @param [in] src  raw data
     * @param [out] dst 16bit unpack data, High four place fill 0
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_raw10_2_ir(const uint8_t *src, uint16_t *dst, int w, int h);

    /**
     *obc_unpackdata_2_depth 
     * @param [in] src  unpack raw data
     * @param [out] dst 16bit depth data
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_unpackdata_2_depth(const uint16_t* src, uint16_t* dst, int w, int h);

    /**
     * raw data_2_depth
     *  totally api,should right config for this paltform 
     * @param [in] src  unpack raw data
     * @param [out] dst 16bit depth data
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_raw_2_depth(const uint8_t *src, uint16_t *dst, int w, int h);

    /**
     * obc_raw_parsin   
     * @param [in] src  raw data
     * @param [out] dst 16bit depth data or Ir data
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_raw_parsing(const uint8_t *src, uint16_t *dst, int size, int* type);



    OBC_API_EXPORT int obc_set_datafmt(uint8_t pixelsize, uint8_t dispbits);

    /**
     *  obc_get_centpoint
     * @param [in] src  raw data
     * @param [in] w   width
     * @param [in] h   height
     * @param [out]  centZ  
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_get_centpoint(const uint16_t* depthBuf, int w, int h, uint16_t* centZ);

    /**
     * debug  api obc_gettable
     * @return  uint16_t *
     */
    OBC_API_EXPORT uint16_t* obc_gettable();

    OBC_API_EXPORT int obc_set_undistortdata(const uint8_t *src, int size, int w, int h);

    OBC_API_EXPORT int obc_undistortdepth(const float *src, uint16_t *dst, int w, int h);


	/**
	* obc_set_mipi_packed
	* @param [in] mipi_packed  是否mipi解包
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT void obc_set_mipi_packed(int mipi_packed);
	/**
	* obc_get_mipi_packed
	* @param [out] mipi_packed  是否mipi解包
	* @return 0: success, < 0 failed
	*/
	OBC_API_EXPORT int  obc_get_mipi_packed(int * p_mipi_packed);


    /**
     *obc_maprealdisp  
     * @param [in] src  raw data
     * @param [in] dst
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT int obc_maprealdisp(const uint16_t* src, float* dst, int w, int h);

    OBC_API_EXPORT int undistort_disparityimpl(const float* disp, int w, int h, float* undis_disp, double* calibParams);

    /**
     * 根据图像分辨率选择处理版本(1280x960和640x480使用Opt版本，其他分辨率使用未优化版本)
     * // int MaxSpeckleSize_1280x960 = 1920;
     * // int MaxSpeckleSize_640x480 = 480;
     * // int MaxSpeckleSize_320x240 = 120;
     * // int MaxSpeckleSize_160x120 = 30;
     * @param [in] opt  no care
     * @param [in] src  image data
     * @param [in] w   width
     * @param [in] h   height
     * @param [in] maxdiff
     * @param [in] specklesize
     * @param [in] new_val
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT void obc_softfilter(int opt, uint16_t* src, int w, int h,  int maxdiff, int specklesize, int new_val);

    OBC_API_EXPORT int obc_set_softfilter(uint8_t opt_enable, uint32_t spSize, uint32_t diff, uint16_t new_val);

    /**
     * 3x3中值滤波
     * @param [in] src  image data
     * @param [in] dst  image data
     * @param [in] w   width
     * @param [in] h   height
     * @return 0: success, < 0 failed
     */
    OBC_API_EXPORT void obc_medianSmooth3x3(unsigned short *src, unsigned short *dst, int w, int h);

    // OBC_API_EXPORT int obc_load_config(char* buf, int size);

    /**
     *  get the frame 's type
     *@param  :  frame data
     *@param size : data size
     *return obdepth_stream_type_t , -ERRNO
     */
    OBC_API_EXPORT int obc_frame_type(const uint8_t *p,int size);

    OBC_API_EXPORT int  dump_s2dconfig( s2dconfig_t* p,int size) ;


	/**
	* @brief  不推荐使用的接口
	*
	*/

	/**
	*@Deprecated  pls use  obc_enable_softfilter
	* @brief  enable filter
	* @param  enable
	* @return none
	*/
	OBC_API_EXPORT void  enable_filter(uint8_t  enable);

	/**
	* @Deprecated
	* @brief  enable_medianfilter
	*
	* @param  enable
	* @return none
	*/
	OBC_API_EXPORT void  enable_medianfilter(uint8_t  enable);



	/**
	* @Deprecated
	* @brief  enable filter
	*
	* @param  enable
	* @return none
	*/
	OBC_API_EXPORT void obc_enable_softfilter(uint8_t en);

#ifdef __cplusplus
}
#endif
#endif
