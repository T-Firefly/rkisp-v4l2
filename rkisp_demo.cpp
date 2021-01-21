/*
 * V4L2 video capture example
 * AUTHOT : Jacob Chen
 * DATA : 2018-02-25
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> /* getopt_long() */
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dlfcn.h>

#include <linux/videodev2.h>
#include <rkisp_control_loop.h>
#include <rkisp_dev_manager.h>
#include <interface/rkcamera_vendor_tags.h>
#include "mediactl.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#undef USE_RGA

#ifdef USE_RGA
#include "rga_ops.h"
#endif

extern "C" {
#define virtual vir
#include <drm.h>
#include <drm_mode.h>
#undef virtual
}

#include <xf86drm.h>
#include <xf86drmMode.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FMT_NUM_PLANES 1

#define BUFFER_COUNT 4

enum io_method {
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
    IO_METHOD_DMABUF,
};

void* _rkisp_engine;

typedef int (*rkisp_init_func)(void** cl_ctx, const char* tuning_file_path,
                               const cl_result_callback_ops_t *callback_ops);
typedef int (*rkisp_prepare_func)(void* cl_ctx,
                     const struct rkisp_cl_prepare_params_s* prepare_params);
typedef int (*rkisp_start_func)(void* cl_ctx);
typedef int (*rkisp_stop_func)(void* cl_ctx);
typedef int (*rkisp_deinit_func)(void* cl_ctx);
typedef int (*rkisp_cl_set_frame_params_func)(const void* cl_ctx,
                     const struct rkisp_cl_frame_metadata_s* frame_params);

struct RKIspFunc {
    void* rkisp_handle;
    rkisp_init_func init_func;
    rkisp_prepare_func prepare_func;
    rkisp_start_func start_func;
    rkisp_stop_func stop_func;
    rkisp_deinit_func deinit_func;
    rkisp_cl_set_frame_params_func set_frame_params_func;
};
struct RKIspFunc _RKIspFunc;

struct RKisp_media_ctl
{
    /* media controller */
    media_device *controller;
    media_entity *isp_subdev;
    media_entity *isp_params_dev;
    media_entity *isp_stats_dev;
    media_entity *sensor_subdev;
};
struct buffer {
        void *start;
        size_t length;
        struct v4l2_buffer v4l2_buf;
};

struct display_buffer {
	void *start;
	size_t length;
	size_t width;
	size_t height;
	int buf_fd;
	int bpp;
};

static char iq_file[255] = "/etc/cam_iq.xml";
static char out_file[255] = "/tmp/fifo";
static char dev_name[255];
static int output_video_flag = 0;
static char output_video[255];
static int width = 640;
static int height = 480;
static int format = V4L2_PIX_FMT_NV12;
static int fd = -1;
static int drm_fd = -1;
static int io = IO_METHOD_MMAP;
static enum v4l2_buf_type buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
struct buffer *buffers;
static unsigned int n_buffers;
static int frame_count = 5;
static float mae_gain = 0.0f;
static float mae_expo = 0.0f;
FILE *fp=NULL;
static int silent;
static unsigned int drm_handle;

int dev_fd;

#ifdef USE_RGA
RockchipRga *rga;
bo_t bo_dst;
rga_info_t rga_info_src;
rga_info_t rga_info_dst;
#endif

struct display_buffer disp_buf;
cv::Mat* mat;

#define DBG(...) do { if(!silent) printf(__VA_ARGS__); } while(0)
#define ERR(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define LIBRKISP "/usr/lib/librkisp.so"

struct control_params_3A
{
    /* used to receive current 3A settings and 3A states
     * place this at first place, so we cast ops back to base type
     */
    cl_result_callback_ops_t _result_cb_ops;
    /* used to set new setting to CL, used by _RKIspFunc.set_frame_params_func */
    rkisp_cl_frame_metadata_s _frame_metas;
    /* to manage the 3A settings, used by _frame_metas */
    CameraMetadata _settings_metadata;
    /* to manage the 3A result settings, used by metadata_result_callback */
    CameraMetadata _result_metadata;
    XCam::Mutex _meta_mutex;
};

static struct control_params_3A* g_3A_control_params = NULL;

static
void metadata_result_callback(const struct cl_result_callback_ops *ops,
                              struct rkisp_cl_frame_metadata_s *result)
{
    camera_metadata_entry entry;
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)ops;

    SmartLock lock(ctl_params->_meta_mutex);
    /* this will clone results to _result_metadata */
    ctl_params->_result_metadata = result->metas;
    DBG("meta callback!\n");
}

/*
 * construct the default camera settings, including the static
 * camera capabilities, infos.
 */
static void construct_default_metas(CameraMetadata* metas)
{
    int64_t exptime_range_ns[2] = {0,30*1000*1000};
    int32_t sensitivity_range[2] = {0,3200};
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_ON;
    uint8_t control_mode = ANDROID_CONTROL_MODE_AUTO;
    uint8_t ae_lock = ANDROID_CONTROL_AE_LOCK_OFF;
    int64_t exptime_ns = 10*1000*1000;
    int32_t sensitivity = 1600;

    metas->update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, exptime_range_ns, 2);
    metas->update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, sensitivity_range, 2);
    metas->update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    metas->update(ANDROID_SENSOR_EXPOSURE_TIME, &exptime_ns, 1);
    metas->update(ANDROID_CONTROL_MODE, &control_mode, 1);
    metas->update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);
    metas->update(ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);
}

static void init_3A_control_params()
{
	return;
    camera_metadata_t* meta;

    meta = allocate_camera_metadata(DEFAULT_ENTRY_CAP, DEFAULT_DATA_CAP);
    assert(meta);
    g_3A_control_params = new control_params_3A();
    assert(g_3A_control_params);
    g_3A_control_params->_result_cb_ops.metadata_result_callback = metadata_result_callback;
    g_3A_control_params->_settings_metadata = meta;
    construct_default_metas(&g_3A_control_params->_settings_metadata);
    g_3A_control_params->_frame_metas.id = 0;
    g_3A_control_params->_frame_metas.metas =
        g_3A_control_params->_settings_metadata.getAndLock();
    g_3A_control_params->_settings_metadata.unlock(g_3A_control_params->_frame_metas.metas);
}

static void deinit_3A_control_params()
{
    if (g_3A_control_params )
        delete g_3A_control_params ;
    g_3A_control_params = NULL;
}

static int rkisp_get_meta_frame_id(void* &engine, int64_t& frame_id) {
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(RKCAMERA3_PRIVATEDATA_EFFECTIVE_DRIVER_FRAME_ID);
    if (!entry.count) {
        DBG("no RKCAMERA3_PRIVATEDATA_EFFECTIVE_DRIVER_FRAME_ID\n");
        return -1;
    }
    frame_id = entry.data.i64[0];
    DBG("meta frame id is %" PRId64 "\n", entry.data.i64[0]);

    return 0;
}

static int rkisp_get_meta_frame_sof_ts(void* &engine, int64_t& sof_ts) {
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(RKCAMERA3_PRIVATEDATA_FRAME_SOF_TIMESTAMP);
    if (!entry.count) {
        DBG("no RKCAMERA3_PRIVATEDATA_FRAME_SOF_TIMESTAMP\n");
        return -1;
    }
    sof_ts = entry.data.i64[0];
    DBG("meta frame timestamp is %" PRId64 "\n", entry.data.i64[0]);

    return 0;
}

// convenient interfaces of 3A, compatible with cifisp
static int rkisp_getAeTime(void* &engine, float &time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_EXPOSURE_TIME);
    if (!entry.count)
        return -1;

    time = entry.data.i64[0] / (1000.0 * 1000.0 * 1000.0);
    DBG("expousre time is %f secs\n", time);

    return 0;
}

static int rkisp_getAeMaxExposureTime(void* &engine, float &time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE);
    if (entry.count != 2)
        return -1;

    time = entry.data.i64[1] / (1000.0 * 1000.0 * 1000.0);
    DBG("expousre max time is %f secs\n", time);

    return 0;
}

static int rkisp_getAeGain(void* &engine, float &gain)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_SENSITIVITY);
    if (!entry.count)
        return -1;

    gain = (float)entry.data.i32[0] / 100;
    DBG("expousre gain is %f\n", gain);

    return 0;
}

static int rkisp_getAeMaxExposureGain(void* &engine, float &gain)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE);
    if (!entry.count)
        return -1;

    gain = entry.data.i32[1] / 100;
    DBG("expousre max gain is %f \n", gain);

    return 0;
}

static int rkisp_setAeMaxExposureTime(void* &engine, float time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int64_t exptime_range_ns[2] = {0,0};

    exptime_range_ns[1] = time * 1000 * 1000 * 1000;
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
                                          exptime_range_ns, 2);
    // should update new settings id
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_setAeMaxExposureGain(void* &engine, float gain)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int32_t sensitivity_range[2] = {0,0};

    sensitivity_range[1] = gain * 100;
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
                                          sensitivity_range, 2);
    // should update new settings id
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_setManualGainAndTime(void* &engine, float hal_gain, float hal_time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int64_t exptime_ns = hal_time * 1000 * 1000 * 1000;
    // set to manual mode
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_OFF;
    // convert to ISO100 unit
    int32_t sensitivity = hal_gain * 100;

    ctl_params->_settings_metadata.update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);
    ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, &exptime_ns, 1);
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);

    return 0;
}

static int rkisp_setAeMode(void* &engine, HAL_AE_OPERATION_MODE mode)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    // set to manual mode
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_OFF;

    if (mode == HAL_AE_OPERATION_MODE_AUTO) {
        ae_mode = ANDROID_CONTROL_AE_MODE_ON;
        ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    } else if (mode == HAL_AE_OPERATION_MODE_MANUAL) {
        camera_metadata_entry entry;
        SmartLock lock(ctl_params->_meta_mutex);

        entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_EXPOSURE_TIME);
        if (!entry.count)
            return -1;
        ctl_params->_settings_metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, entry.data.i64, 1);

        entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_SENSITIVITY);
        if (!entry.count)
            return -1;
        ctl_params->_settings_metadata.update(ANDROID_SENSOR_SENSITIVITY, entry.data.i32, 1);

        ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    } else {
        ERR("unsupported ae mode %d\n", mode);
        return -1;
    }
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);

    return 0;
}

enum HAL_AE_STATE {
    HAL_AE_STATE_UNSTABLE,
    HAL_AE_STATE_STABLE,
};

static int rkisp_getAeState(void* &engine, enum HAL_AE_STATE &ae_state)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_CONTROL_AE_STATE);
    if (!entry.count)
        return -1;

    switch (entry.data.u8[0]) {
    case ANDROID_CONTROL_AE_STATE_SEARCHING :
        ae_state = HAL_AE_STATE_UNSTABLE;
        break;
    default:
        ae_state = HAL_AE_STATE_STABLE;
        break;
    }

    return 0;
}

static int rkisp_set3ALocks(void* &engine, int locks)
{
    // TODO: only support ae lock now
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    uint8_t ae_lock = locks & HAL_3A_LOCKS_EXPOSURE ?
        ANDROID_CONTROL_AE_LOCK_ON : ANDROID_CONTROL_AE_LOCK_OFF;

    ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_get3ALocks(void* &engine, int& curLocks)
{
    // TODO: only support ae lock now
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_CONTROL_AE_LOCK);
    if (!entry.count)
        return -1;

    if (entry.data.u8[0] == ANDROID_CONTROL_AE_LOCK_ON)
        curLocks |= HAL_3A_LOCKS_EXPOSURE;

    return 0;
}

static void errno_exit(const char *s)
{
        ERR("%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

#if 0
static int init_drm()
{
    int drm_fd = drmOpen("rockchip", NULL);
    if (drm_fd < 0) {
        ERR("failed to open rockchip drm: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    return drm_fd;
}

static void deinit_drm(int drm_fd)
{
    drmClose(drm_fd);
}

static void* get_drm_buf(int drm_fd, int width, int height, int bpp)
{
    struct drm_mode_create_dumb alloc_arg;
    struct drm_mode_map_dumb mmap_arg;
    struct drm_mode_destroy_dumb destory_arg;
    int ret;
    void *map;

    CLEAR(alloc_arg);
    alloc_arg.bpp = bpp;
    alloc_arg.width = width;
    alloc_arg.height = height;
#define ROCKCHIP_BO_CONTIG  1
#define ROCKCHIP_BO_CACHABLE (1<<1)
    alloc_arg.flags = ROCKCHIP_BO_CONTIG | ROCKCHIP_BO_CACHABLE;

    ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &alloc_arg);
    if (ret) {
        ERR("failed to create dumb buffer: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    CLEAR(mmap_arg);
    mmap_arg.handle = alloc_arg.handle;

    ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mmap_arg);
    if (ret) {
        ERR("failed to create map dumb: %s\n", strerror(errno));
        ret = -EINVAL;
        goto destory_dumb;
    }

    map = mmap(0, alloc_arg.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, mmap_arg.offset);
    if (map == MAP_FAILED) {
        ERR("failed to mmap buffer: %s\n", strerror(errno));
        ret = -EINVAL;
        goto destory_dumb;
    }

    assert(alloc_arg.size == width * height * bpp / 8);

destory_dumb:
    CLEAR(destory_arg);
    destory_arg.handle = alloc_arg.handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destory_arg);
    if (ret)
        return NULL;

    return map;
}

static void* get_drm_fd(int drm_fd, int width, int height, int bpp, int *export_dmafd)
{
    struct drm_mode_create_dumb alloc_arg;
    struct drm_mode_map_dumb mmap_arg;
    struct drm_mode_destroy_dumb destory_arg;
    int ret;
    void *map;

    CLEAR(alloc_arg);
    alloc_arg.bpp = bpp;
    alloc_arg.width = width;
    alloc_arg.height = height;
#define ROCKCHIP_BO_CONTIG  1
#define ROCKCHIP_BO_CACHABLE (1<<1)
    alloc_arg.flags = ROCKCHIP_BO_CONTIG | ROCKCHIP_BO_CACHABLE;

    ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &alloc_arg);
    if (ret) {
        ERR("failed to create dumb buffer: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    CLEAR(mmap_arg);
    mmap_arg.handle = alloc_arg.handle;
    ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mmap_arg);
    if (ret) {
        ERR("failed to create map dumb: %s\n", strerror(errno));
        ret = -EINVAL;
        goto destory_dumb;
    }

    map = mmap(0, alloc_arg.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, mmap_arg.offset);
    if (map == MAP_FAILED) {
        ERR("failed to mmap buffer: %s\n", strerror(errno));
        ret = -EINVAL;
        goto destory_dumb;
    }

    assert(alloc_arg.size == width * height * bpp / 8);
    ret = drmPrimeHandleToFD(drm_fd, alloc_arg.handle, 0, export_dmafd);
    if (ret) {
        ERR("failed to export fd: %s\n", strerror(errno));
        ret = -EINVAL;
        goto destory_dumb;
    }

    drm_handle = alloc_arg.handle;

    return map;

destory_dumb:
    CLEAR(destory_arg);
    destory_arg.handle = alloc_arg.handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destory_arg);

    return NULL;
}
#endif

static int xioctl(int fh, int request, void *arg)
{
        int r;
        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);
        return r;
}

static void process_buffer(struct buffer* buff, int size)
{

	if (fp) {
		fwrite(buff->start, size, 1, fp);
		fflush(fp);
	}

    if (output_video_flag && write(dev_fd, buff->start, size) != size) {
        errno_exit("process_buffer read error");
    }

#ifdef USE_RGA
	rga_info_src.virAddr = buff->start;
	rga->RkRgaBlit(&rga_info_src, &rga_info_dst, NULL);

	cv::imshow("video", *mat);
#else
	cv::Mat yuvmat(cv::Size(width, height*3/2), CV_8UC1, buff->start);
    cv::Mat rgbmat(cv::Size(width, height), CV_8UC3);
    cv::cvtColor(yuvmat, rgbmat, CV_YUV2BGR_NV12);
	cv::imshow("video", rgbmat);
#endif

	cv::waitKey(1);

}

static int read_frame()
{
        struct v4l2_buffer buf;
        int i, bytesused;

        CLEAR(buf);

        buf.type = buf_type;
		buf.memory = V4L2_MEMORY_MMAP;

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
            struct v4l2_plane planes[FMT_NUM_PLANES];
            buf.m.planes = planes;
            buf.length = FMT_NUM_PLANES;
        }

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
                errno_exit("VIDIOC_DQBUF");

        i = buf.index;

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type)
            bytesused = buf.m.planes[0].bytesused;
        else
            bytesused = buf.bytesused;
		process_buffer(&(buffers[i]), bytesused);
        DBG("bytesused %d\n", bytesused);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

        return 1;
}

static unsigned long get_time(void)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	return (ts.tv_sec * 1000 + ts.tv_usec / 1000);
}

static void mainloop(void)
{
        unsigned int count = 1;
        float exptime, expgain;
        int64_t frame_id, frame_sof;
        pthread_t display_thread_id;

#if 0
        if (mae_gain > 0 && mae_expo > 0)
            rkisp_setManualGainAndTime((void*&)g_3A_control_params, mae_gain, mae_expo);
        else
            rkisp_setAeMode((void*&)g_3A_control_params, HAL_AE_OPERATION_MODE_AUTO);
#endif

        //pthread_create(&display_thread_id, NULL, RgaProcessThread, dec);
        unsigned long read_start_time, read_end_time;
        while (1) {

            DBG("No.%d\n", count);        //显示当前帧数目
            if(count++ == frame_count)
                break;

			read_start_time = get_time();
            // examples show how to use 3A interfaces
#if 0
            rkisp_getAeTime((void*&)g_3A_control_params, exptime);
            rkisp_getAeGain((void*&)g_3A_control_params, expgain);
            rkisp_getAeMaxExposureGain((void*&)g_3A_control_params, expgain);
            rkisp_getAeMaxExposureTime((void*&)g_3A_control_params, exptime);
            rkisp_get_meta_frame_id((void*&)g_3A_control_params, frame_id);
            rkisp_get_meta_frame_sof_ts((void*&)g_3A_control_params, frame_sof);
#endif
            read_frame();
			read_end_time = get_time();
			DBG("take time %lu ms\n",read_end_time - read_start_time);
        }
        DBG("\nREAD AND SAVE DONE!\n");
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

    	if (_RKIspFunc.stop_func != NULL) {
    	    DBG("stop rkisp engine\n");
    	    _RKIspFunc.stop_func(_rkisp_engine);
    	}
        type = buf_type;
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");
}

#define MAX_MEDIA_INDEX 64
static struct media_device* __rkisp_get_media_dev_by_vnode(const char* vnode) {
    char sys_path[64];
    struct media_device *device = NULL;
    uint32_t nents, j, i = 0;
    FILE *fp;
    int ret;

    while (i < MAX_MEDIA_INDEX) {
        snprintf (sys_path, 64, "/dev/media%d", i++);
        fp = fopen (sys_path, "r");
        if (!fp)
          continue;
        fclose (fp);

        device = media_device_new (sys_path);

        /* Enumerate entities, pads and links. */
        media_device_enumerate (device);

        nents = media_get_entities_count (device);
        for (j = 0; j < nents; ++j) {
          struct media_entity *entity = media_get_entity (device, j);
          const char *devname = media_entity_get_devname (entity);
          if (NULL != devname) {
            if (!strcmp (devname, vnode)) {
                  goto out;
            }
          }
        }
        media_device_unref (device);
    }

out:
    return device;
}


static void start_capturing(void)
{
        unsigned int i;
        struct RKisp_media_ctl rkisp;
        enum v4l2_buf_type type;

#if 0
    	if (_RKIspFunc.init_func != NULL) {
			_RKIspFunc.init_func(&_rkisp_engine, iq_file,
                               (cl_result_callback_ops_t*)(g_3A_control_params));
		}
    	if (_RKIspFunc.prepare_func != NULL) {
			struct rkisp_cl_prepare_params_s params={0};
            int nents;

            rkisp.controller =
                __rkisp_get_media_dev_by_vnode (dev_name);
            if (!rkisp.controller)
                errno_exit(
                    "Can't find controller, maybe use a wrong video-node or wrong permission to media node");
            rkisp.isp_subdev =
              media_get_entity_by_name (rkisp.controller, "rkisp1-isp-subdev",
                                        strlen("rkisp1-isp-subdev"));
            rkisp.isp_params_dev =
              media_get_entity_by_name (rkisp.controller, "rkisp1-input-params",
                                        strlen("rkisp1-input-params"));
            rkisp.isp_stats_dev =
              media_get_entity_by_name (rkisp.controller, "rkisp1-statistics",
                                        strlen("rkisp1-statistics"));
            /* TODO: Get ACTIVE subdev sensor */
            nents = media_get_entities_count (rkisp.controller);
            rkisp.sensor_subdev = NULL;
            for (i = 0; i < nents; ++i) {
                struct media_entity *e;
                const struct media_entity_desc *ef;

                e = media_get_entity(rkisp.controller, i);
                ef = media_entity_get_info(e);
                if (ef->type == MEDIA_ENT_T_V4L2_SUBDEV_SENSOR) {
                    rkisp.sensor_subdev = e;
                    break;
                }
            }
            assert(rkisp.sensor_subdev);

            params.isp_sd_node_path = media_entity_get_devname (rkisp.isp_subdev);
            params.isp_vd_params_path = media_entity_get_devname (rkisp.isp_params_dev);
            params.isp_vd_stats_path = media_entity_get_devname (rkisp.isp_stats_dev);
            params.sensor_sd_node_path = media_entity_get_devname (rkisp.sensor_subdev);
            params.staticMeta =
                g_3A_control_params->_settings_metadata.getAndLock();
            /*
            // isp subdev node path
            params.isp_sd_node_path="/dev/v4l-subdev0";
            // isp params video node path
            params.isp_vd_params_path="/dev/video3";
            // isp statistics video node path
            params.isp_vd_stats_path="/dev/video2";
            // camera sensor subdev node path
            params.sensor_sd_node_path="/dev/v4l-subdev2";
            */
			_RKIspFunc.prepare_func(_rkisp_engine, &params);
            g_3A_control_params->_settings_metadata.unlock(params.staticMeta);
            media_device_unref (rkisp.controller);
		}

        // set initial user params
        if (_RKIspFunc.set_frame_params_func != NULL) {
            _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                             &g_3A_control_params->_frame_metas);
        }

    	if (_RKIspFunc.start_func != NULL) {
    	    DBG("device manager start, capture dev fd: %d\n", fd);
    	    _RKIspFunc.start_func(_rkisp_engine);
    	    DBG("device manager isp_init\n");

    	    if (_rkisp_engine == NULL) {
    	        ERR("rkisp_init engine failed\n");
    	    } else {
    	        DBG("rkisp_init engine succeed\n");
    	    }
    	}
#endif
        for (i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = buf_type;
				buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
                    struct v4l2_plane planes[FMT_NUM_PLANES];

                    buf.m.planes = planes;
                    buf.length = FMT_NUM_PLANES;
                }
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
        }
        type = buf_type;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
}

static void uninit_device(void)
{
        unsigned int i;
        struct drm_mode_destroy_dumb destory_arg;

        for (i = 0; i < n_buffers; ++i) {
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                        errno_exit("munmap");
        }

        free(buffers);

        if (_RKIspFunc.deinit_func != NULL) {
            _RKIspFunc.deinit_func(_rkisp_engine);
            //deinit_3A_control_params();
        }

        dlclose(_RKIspFunc.rkisp_handle);
#if 0
        if (drm_fd != -1)
            deinit_drm(drm_fd);
#endif
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = BUFFER_COUNT;
        req.type = buf_type;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        ERR("%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                ERR("Insufficient buffer memory on %s\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = (struct buffer*)calloc(req.count, sizeof(*buffers));

        if (!buffers) {
                ERR("Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;
                struct v4l2_plane planes[FMT_NUM_PLANES];
                CLEAR(buf);
                CLEAR(planes);

                buf.type = buf_type;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = n_buffers;

                if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
                    buf.m.planes = planes;
                    buf.length = FMT_NUM_PLANES;
                }

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
                    buffers[n_buffers].length = buf.m.planes[0].length;
                    buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.m.planes[0].length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.planes[0].m.mem_offset);
                } else {
                    buffers[n_buffers].length = buf.length;
                    buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);
                }

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_display_buf(int buffer_size, int width, int height)
{
    int bpp;
    int export_dmafd;
	//if (drm_fd == -1)
		//drm_fd = init_drm();

    //disp_buf.start = get_drm_fd(drm_fd, width, height, 24, &export_dmafd);
	bpp = buffer_size * 8 / width / height;
	disp_buf.length= buffer_size;
	disp_buf.width = width;
	disp_buf.height= height;
	disp_buf.bpp   = bpp;
	//disp_buf.buf_fd= export_dmafd;

	//mat = new cv::Mat(cv::Size(width, height), CV_8UC3, disp_buf.start);
#ifdef USE_RGA
	//alloc a buffer for rga input
//dst
	rga->RkRgaGetAllocBuffer(&bo_dst, disp_buf.width, disp_buf.height, 32);
	rga->RkRgaGetBufferFd(&bo_dst, &(rga_info_dst.fd));

	set_rect_size(&(rga_info_dst.rect), disp_buf.width, disp_buf.height);
	set_rect_crop(&(rga_info_dst.rect), 0, 0, disp_buf.width, disp_buf.height);
	set_rect_format(&(rga_info_dst.rect), V4l2ToRgaFormat(V4L2_PIX_FMT_RGB24));

	rga->RkRgaGetMmap(&bo_dst);
	mat = new cv::Mat(cv::Size(width, height), CV_8UC3, bo_dst.ptr);

//src
	set_rect_size(&(rga_info_src.rect), width, height);
	set_rect_crop(&(rga_info_src.rect), 0, 0, width, height);
	set_rect_format(&(rga_info_src.rect), V4l2ToRgaFormat(format));
#endif
	cv::namedWindow("video");
}

#ifdef USE_RGA
static void init_rga(struct display_buffer* disp_buf)
{
	rga = new RockchipRga();
	rga->RkRgaInit();

	memset(&rga_info_src, 0, sizeof(rga_info_t));
	rga_info_src.mmuFlag = 1;
	rga_info_src.fd = -1;

	memset(&rga_info_dst, 0, sizeof(rga_info_t));
	rga_info_dst.mmuFlag = 1;
	rga_info_dst.fd = -1;
}
#endif

static void init_device(void)
{
        struct v4l2_capability cap;
        struct v4l2_format fmt;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        ERR("%s is no V4L2 device\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
                !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
            ERR("%s is not a video capture device, capabilities: %x\n",
                         dev_name, cap.capabilities);
                exit(EXIT_FAILURE);
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                ERR("%s does not support streaming i/o\n",
                    dev_name);
                exit(EXIT_FAILURE);
        }

        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
            buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
            buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        CLEAR(fmt);
        fmt.type = buf_type;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = format;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

		init_mmap();

#ifdef USE_RGA
		init_rga(&disp_buf);
#endif
        init_display_buf(fmt.fmt.pix.sizeimage, width, height);

		//INIT RKISP
        _RKIspFunc.rkisp_handle = dlopen(LIBRKISP, RTLD_NOW);
    	if (_RKIspFunc.rkisp_handle == NULL) {
            ERR("open %s failed\n", LIBRKISP);
    	} else {
            DBG("open %s successed\n", LIBRKISP);
    	    _RKIspFunc.init_func=(rkisp_init_func)dlsym(_RKIspFunc.rkisp_handle, "rkisp_cl_init");
    	    _RKIspFunc.prepare_func=(rkisp_prepare_func)dlsym(_RKIspFunc.rkisp_handle, "rkisp_cl_prepare");
    	    _RKIspFunc.start_func=(rkisp_start_func)dlsym(_RKIspFunc.rkisp_handle, "rkisp_cl_start");
    	    _RKIspFunc.stop_func=(rkisp_stop_func)dlsym(_RKIspFunc.rkisp_handle, "rkisp_cl_stop");
    	    _RKIspFunc.deinit_func=(rkisp_deinit_func)dlsym(_RKIspFunc.rkisp_handle, "rkisp_cl_deinit");
            _RKIspFunc.set_frame_params_func=(rkisp_cl_set_frame_params_func)dlsym(_RKIspFunc.rkisp_handle,
                                                                      "rkisp_cl_set_frame_params");
    	    if (_RKIspFunc.start_func == NULL) {
    	        ERR("func rkisp_start not found.\n");
    	        const char *errmsg;
    	        if ((errmsg = dlerror()) != NULL) {
    	            ERR("dlsym rkisp_start fail errmsg: %s\n", errmsg);
    	        }
    	    } else {
                DBG("dlsym rkisp_start success\n");
    	    }
            init_3A_control_params();
    	}

}

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        fd = open(dev_name, O_RDWR /* required */ /*| O_NONBLOCK*/, 0);

        if (-1 == fd) {
                ERR("Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

void parse_args(int argc, char **argv)
{
    int c;
    int digit_optind = 0;

    while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] = {
            {"width",    required_argument, 0, 'w' },
            {"height",   required_argument, 0, 'h' },
            {"format",   required_argument, 0, 'f' },
            {"iqfile",   required_argument, 0, 'i' },
            {"device",   required_argument, 0, 'd' },
            {"output_dev",   required_argument, 0, 'D' },
            {"output",   required_argument, 0, 'o' },
            {"count",    required_argument, 0, 'c' },
            {"expo",     required_argument, 0, 'e' },
            {"gain",     required_argument, 0, 'g' },
            {"help",     no_argument,       0, 'p' },
            {"silent",   no_argument,       0, 's' },
            {0,          0,                 0,  0  }
        };

        c = getopt_long(argc, argv, "w:h:m:f:i:d:D:o:c:e:g:ps",
        long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
        case 'c':
            frame_count = atoi(optarg);
            break;
        case 'e':
            mae_expo = atof(optarg);
            DBG("target expo: %f\n", mae_expo);
            break;
        case 'g':
            mae_gain = atof(optarg);
            DBG("target gain: %f\n", mae_gain);
            break;
        case 'w':
            width = atoi(optarg);
            break;
        case 'h':
            height = atoi(optarg);
            break;
        case 'f':
            format = v4l2_fourcc(optarg[0], optarg[1], optarg[2], optarg[3]);
            break;
        case 'i':
            strcpy(iq_file, optarg);
            break;
        case 'd':
            strcpy(dev_name, optarg);
            break;
        case 'D':
            output_video_flag = 1;
            strcpy(output_video, optarg);
            break;
        case 'o':
            strcpy(out_file, optarg);
            break;
        case 's':
            silent = 1;
            break;
        case '?':
        case 'p':
            ERR("Usage: %s to capture rkisp1 frames\n"
            "         --width,  default 640,             optional, width of image\n"
            "         --height, default 480,             optional, height of image\n"
            "         --memory, default mmap,            optional, use 'mmap' or 'drm' to alloc buffers\n"
            "         --format, default NV12,            optional, fourcc of format\n"
            "         --count,  default    5,            optional, how many frames to capture\n"
            "         --iqfile, default /etc/cam_iq.xml, optional, camera IQ file\n"
            "         --device,                          required, path of video device\n"
            "         --output_dev,                      required, path of output video device\n"
            "         --output, default null             optional, output file path\n"
            "         --gain,   default 0,               optional\n"
            "         --expo,   default 0,               optional\n"
            "                   Manually AE is enable only if --gain and --expo are not zero\n"
            "         --silent,                          optional, subpress debug log\n",
            argv[0]);
            exit(-1);

        default:
            ERR("?? getopt returned character code 0%o ??\n", c);
        }
    }

    if (strlen(dev_name) == 0) {
        ERR("arguments -device are required\n");
        exit(-1);
    }
}

void
sysfail(char *msg)
{
	perror(msg);
	exit(1);
}

#define vidioc(op, arg) \
	if (ioctl(dev_fd, VIDIOC_##op, arg) == -1) \
		sysfail(#op); \
	else

void
init_output_video(char * name)
{
	struct v4l2_format v;

	dev_fd = open(name, O_RDWR);
	if (dev_fd == -1) sysfail(name);
	v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	vidioc(G_FMT, &v);
	v.fmt.pix.width = width;//frame_width;
	v.fmt.pix.height = height;//frame_height;
	v.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
	v.fmt.pix.sizeimage = (width * height*3/2);//frame_bytes = 3 * frame_width * frame_height / 2;
	vidioc(S_FMT, &v);
    //close(dev_fd);
}

int main(int argc, char **argv)
{
    parse_args(argc, argv);
/*
    if (strlen(out_file) != 0) {
        init_output_video(out_file);
        if ((fp = fopen(out_file, "w")) == NULL) {
            perror("Creat file failed");
            exit(0);
        }
    }
*/
    if(output_video_flag)
        init_output_video(output_video);

    open_device();
    init_device();
    start_capturing();
    mainloop();
    if (fp)
        fclose(fp);
    stop_capturing();
    uninit_device();
    close_device();
    return 0;
}
