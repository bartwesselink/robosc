import { parse as parseMsgDef } from '@foxglove/rosmsg';
import { RosNode } from '@foxglove/ros2';
import { getNetworkInterfaces, UdpSocketNode } from '@foxglove/ros2/nodejs';
import * as vscode from 'vscode';

export const startListener = async (fetchWebview: () => vscode.Webview) => {
    const ros = new RosNode({
        name: 'visualizer',
        udpSocketCreate: UdpSocketNode.Create,
        getNetworkInterfaces,
    });

    await ros.start();

    const msgDefinition = parseMsgDef('string data', { ros2: true });

    const subscription = ros.subscribe({
        topic: '/state',
        dataType: 'std_msgs/msg/String',
        msgDefinition,
    });

    subscription.on('message', (timestamp, msg, data, publisher) => {
        const strMsg = msg as { data: string };
        const parsed = JSON.parse(strMsg.data);

        const webview = fetchWebview();
        webview.postMessage({ command: 'visualize', data: parsed});
    });

    return ros;
};