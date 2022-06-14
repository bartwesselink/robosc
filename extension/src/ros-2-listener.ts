import { parse as parseMsgDef } from '@foxglove/rosmsg';
import { RosNode } from '@foxglove/ros2';
import { getNetworkInterfaces, UdpSocketNode } from '@foxglove/ros2/nodejs';
import * as vscode from 'vscode';
import { Configuration } from './models';

export const startRos2Listener = async (fetchWebview: () => vscode.Webview, configuration: Configuration) => {
    const ros = new RosNode({
        name: 'visualizer',
        udpSocketCreate: UdpSocketNode.Create,
        getNetworkInterfaces,
        domainId: configuration.ros2?.domainId ?? 0,
    });

    await ros.start();

    const msgDefinition = parseMsgDef('string data', { ros2: true });

    const subscription = ros.subscribe({
        topic: '/state',
        dataType: 'std_msgs/msg/String',
        msgDefinition,
    });

    subscription.on('message', (timestamp: any, msg: any, data: any, publisher: any) => {
        const strMsg = msg as { data: string };
        const parsed = JSON.parse(strMsg.data);

        const webview = fetchWebview();
        webview.postMessage({ command: 'visualize', data: parsed});
    });

    console.log('Listening for ROS2 communication...');

    return ros;
};