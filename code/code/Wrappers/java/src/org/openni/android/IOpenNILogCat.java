package org.openni.android;

/**
 * Created by lixiaobin.
 * DateTime: 2020/1/7 9:16
 * Description:Redirect logcat.The user can implement this interface and call the {@link org.openni.OpenNILog}  class to set the redirection instance object {@link IOpenNILogCat}.
 */
public interface IOpenNILogCat {

    int verbose(String tag, String msg);

    int debug(String tag, String msg);

    int info(String tag, String msg);

    int warning(String tag, String msg);

    int error(String tag, String msg);

}
