/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
package org.openni.android.tools.niviewer;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.ViewGroup.LayoutParams;
import android.widget.LinearLayout;

import org.openni.Device;
import org.openni.DeviceInfo;
import org.openni.OpenNI;
import org.openni.Recorder;
import org.openni.android.OpenNIHelper;

import org.openni.android.tools.niviewer.StreamView;

import static java.lang.Thread.sleep;

public class NiViewerActivity extends Activity
		implements OpenNIHelper.DeviceOpenListener {
	
	private static final String TAG = "NiViewer";
	private OpenNIHelper mOpenNIHelper;
	private boolean mDeviceOpenPending = false;
	private Device mDevice;
	private Recorder mRecorder;
	private String mRecordingName;
	private String mRecording;
	private LinearLayout mStreamsContainer;
	private int mActiveDeviceID = -1;
	private BroadcastReceiver mUsbReceiver;
	

	private final int OB_VID = 0x2BC5;
	private final int OB_MX400_START_PID = 0x0400;
	private final int OB_MX400_END_PID = 0x04FF;
	private final int OB_MX6000_START_PID = 0x0601;
	private final int OB_MX6000_END_PID = 0x06FF;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		Log.d(TAG, "onCreate");
		mOpenNIHelper = new OpenNIHelper(this);
		OpenNI.setLogAndroidOutput(true);
		OpenNI.setLogMinSeverity(0);
		//OpenNI.initialize();
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_niviewer);
		mStreamsContainer = (LinearLayout)findViewById(R.id.streams_container);
		onConfigurationChanged(getResources().getConfiguration());
		mUsbReceiver = new BroadcastReceiver() {
			public void onReceive(Context context, Intent intent) {
				String action = intent.getAction();
				if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
					Log.d(TAG,"usb out");
					UsbDevice usbDevice = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (usbDevice != null) {
						if (usbDevice.getVendorId() == OB_VID && ((usbDevice.getProductId() <= OB_MX400_END_PID && usbDevice.getProductId() >= OB_MX400_START_PID) || (usbDevice.getProductId() <= OB_MX6000_END_PID && usbDevice.getProductId() >= OB_MX6000_START_PID))) {
							Log.d(TAG, "PID:" + usbDevice.getProductId()+"VID:"+usbDevice.getVendorId());

							//stopRecording();

							for (StreamView streamView : getStreamViews()) {
								streamView.stop();
								mStreamsContainer.removeView(streamView);
							}

							if (mDevice != null) {
								mDevice.close();
								mDevice=null;
							}
							OpenNI.shutdown();
						}
					}
				}else if(UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
					Log.d(TAG, "usb in");
					UsbDevice usbDevice = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (usbDevice != null) {
						if (usbDevice.getVendorId() == OB_VID && ((usbDevice.getProductId() <= OB_MX400_END_PID && usbDevice.getProductId() >= OB_MX400_START_PID) || (usbDevice.getProductId() <= OB_MX6000_END_PID && usbDevice.getProductId() >= OB_MX6000_START_PID))) {
							Log.d(TAG, "PID:" + usbDevice.getProductId()+"VID:"+usbDevice.getVendorId());

							try {
								sleep(500);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
							mDeviceOpenPending = true;
							OpenNI.setLogAndroidOutput(true);
							OpenNI.setLogMinSeverity(0);
							UsbDevice device = mOpenNIHelper.getUsbDevice();
							if (device == null) {
								showAlertAndExit("there is no device");
								return;
							}
							mOpenNIHelper.requestUSBPermission(device);
						}
					}
				}
			}
		};

		IntentFilter usbDeviceStateFilter = new IntentFilter();
		usbDeviceStateFilter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
		usbDeviceStateFilter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
		registerReceiver(mUsbReceiver, usbDeviceStateFilter);

	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.niviewer_menu, menu);
		return true;
	}
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
			case R.id.record:
				toggleRecording(item);
				return true;
			case R.id.add_stream:
				addStream();
				return true;
			case R.id.device:
				switchDevice();
				return true;
			default:
				return super.onOptionsItemSelected(item);
		}
	}

	@Override
	protected void onDestroy() {
		Log.d(TAG, "onDestroy");
		super.onDestroy();
		unregisterReceiver(mUsbReceiver);
		mOpenNIHelper.shutdown();
		OpenNI.shutdown();
	}
	
	@Override 
	protected void onStart() {
		Log.d(TAG, "onStart");
		super.onStart();
		
		final android.content.Intent intent = getIntent ();

		if (intent != null) {
			final android.net.Uri data = intent.getData ();
			if (data != null) {
				mRecording = data.getEncodedPath ();
				Log.d(TAG, "Will open file " + mRecording);
			}
		}
	}

	@Override
	protected void onResume() {
		Log.d(TAG, "onResume");

		super.onResume();
		
		// onResume() is called after the USB permission dialog is closed, in which case, we don't want
		// to request device permissions again
		if (mDeviceOpenPending) {
			return;
		}

		mDeviceOpenPending = true;
        List<DeviceInfo> devices = OpenNI.enumerateDevices();
		if (!devices.isEmpty()) {
			OpenNI.shutdown();
		}
		UsbDevice device=mOpenNIHelper.getUsbDevice();
		if(device==null){
			showAlertAndExit("there is no device");
			return;
		}
		mOpenNIHelper.setDeviceOpenListener(this);
		mOpenNIHelper.requestUSBPermission(device);
		
	}
	
	@Override
	public void onConfigurationChanged(Configuration config) {
		Log.d(TAG, "onConfigurationChanged");
		
		if (Configuration.ORIENTATION_PORTRAIT == config.orientation) {
			mStreamsContainer.setOrientation(LinearLayout.VERTICAL);
		} else {
			mStreamsContainer.setOrientation(LinearLayout.HORIZONTAL);
		}

		//Re-insert each view to force correct display (forceLayout() doesn't work)
		for (StreamView streamView : getStreamViews()) {
			mStreamsContainer.removeView(streamView);
			setStreamViewLayout(streamView, config);
			mStreamsContainer.addView(streamView);
		}
		
		super.onConfigurationChanged(config);
	}
	
	private void showAlert(String message) {
		AlertDialog.Builder builder = new AlertDialog.Builder(this);
		builder.setMessage(message);
		builder.show();
	}
	
	private void showAlertAndExit(String message) {
		AlertDialog.Builder builder = new AlertDialog.Builder(this);
		builder.setMessage(message);
		builder.setNeutralButton("OK", new DialogInterface.OnClickListener() {
			public void onClick(DialogInterface dialog, int which) {
				finish();
			}
		});
		builder.show();
	}

	//@Override
	public void onDeviceOpened(Device aDevice) {
		Log.d(TAG, "Permission granted for device " + aDevice.getDeviceInfo().getUri());
		
		mDeviceOpenPending = false;

		mDevice = aDevice;
		
		//Find device ID
		List<DeviceInfo> devices = OpenNI.enumerateDevices();
		for(int i=0; i < devices.size(); i++)
		{
			if(devices.get(i).getUri().equals(mDevice.getDeviceInfo().getUri())){
				mActiveDeviceID = i;
				break;
			}
		}
		
		/*for (StreamView streamView : getStreamViews()) {
			streamView.setDevice(mDevice);
		}*/
		
		mStreamsContainer.requestLayout();
		addStream();
	}
	
	private void setStreamViewLayout(StreamView streamView, Configuration config) {
		LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(
				LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT); 
		if (config.orientation == Configuration.ORIENTATION_PORTRAIT) {
			params.width = LayoutParams.WRAP_CONTENT;
			params.height = 0;
		} else {
			params.width = 0;
			params.height = LayoutParams.WRAP_CONTENT;
		}

		params.weight = 1;
		params.gravity = Gravity.CENTER;
		streamView.setLayoutParams(params);
	}
	
	private void addStream() {
		StreamView streamView = new StreamView(this);
		setStreamViewLayout(streamView, getResources().getConfiguration());
		
		streamView.setDevice(mDevice);
		mStreamsContainer.addView(streamView);
		mStreamsContainer.requestLayout();
	}

	@SuppressLint("SimpleDateFormat")
	private void toggleRecording(MenuItem item) {
		if (mRecorder == null) {
			mRecordingName = Environment.getExternalStorageDirectory().getPath() +
					"/" + new SimpleDateFormat("yyyyMMddHHmmss").format(new Date()) + ".oni";
			
			try {
				mRecorder = Recorder.create(mRecordingName);
				for (StreamView streamView : getStreamViews()) {
					mRecorder.addStream(streamView.getStream(), true);
				}
				mRecorder.start();
			} catch (RuntimeException ex) {
				mRecorder = null;
				showAlert("Failed to start recording: " + ex.getMessage());
				return;
			}
			
			item.setTitle(R.string.stop_record);
		} else {
			stopRecording();
			item.setTitle(R.string.start_record);
		}
	}
	
	private void stopRecording() {
		if (mRecorder != null) {
			mRecorder.stop();
			mRecorder.destroy();
			mRecorder = null;

			showAlert("Recording saved to: " + mRecordingName);
			mRecordingName = null;
		}
	}
	
	private void switchDevice() {
		List<DeviceInfo> devices = OpenNI.enumerateDevices();
		if (devices.isEmpty()) {
			showAlertAndExit("No OpenNI-compliant device found.");
			return;
		}

		new DeviceSelectDialog().showDialog(devices, mActiveDeviceID, this);
	}

	public void openDevice(String deviceURI) {
		if (mDeviceOpenPending) {
			return;
		}

		stopRecording();

		for (StreamView streamView : getStreamViews()) {
			streamView.stop();
			mStreamsContainer.removeView(streamView);
		}
		
		if (mDevice != null) {
			mDevice.close();
			mDevice=null;
		}

		mDeviceOpenPending = true;
		mOpenNIHelper.requestDeviceOpen(deviceURI, this);
	}

	@Override
	public void onDeviceOpenFailed(String uri) {
		Log.e(TAG, "Failed to open device " + uri);
		mDeviceOpenPending = false;
		showAlertAndExit("Failed to open device");
	}

	@Override
	protected void onPause() {
		Log.d(TAG, "onPause");
		super.onPause();
		
		// onPause() is called just before the USB permission dialog is opened, in which case, we don't
		// want to shutdown OpenNI
		if (mDeviceOpenPending)
			return;
	}

	@Override
	protected void onStop() {
		super.onStop();
		stopRecording();

		for (StreamView streamView : getStreamViews()) {
			streamView.stop();
			mStreamsContainer.removeView(streamView);
		}

		if (mDevice != null) {
			mDevice.close();
			mDevice = null;
		}

	}
	
	private List<StreamView> getStreamViews() {
		int count = mStreamsContainer.getChildCount();
		ArrayList<StreamView> list = new ArrayList<StreamView>(count);
		for (int i = 0; i < count; ++i) {
			StreamView view = (StreamView)mStreamsContainer.getChildAt(i);
			list.add(view);
		}
		return list;
	}

	@Override
	public void onPointerCaptureChanged(boolean hasCapture) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onDeviceOpened(UsbDevice usbDevice) {
		// TODO Auto-generated method stub
		onDeviceOpened(Device.open());
	}
}
