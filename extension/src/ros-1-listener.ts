import { RosNode } from "@foxglove/ros1";
import { getEnvVar, getHostname, getNetworkInterfaces, getPid, TcpSocketNode } from "@foxglove/ros1/nodejs";
import { HttpServerNodejs } from "@foxglove/xmlrpc/nodejs";
import * as vscode from 'vscode';
import { Configuration } from './models';

export const startRos1Listener = async (fetchWebview: () => vscode.Webview, configuration: Configuration) => {
    const ros = new RosNode({
        name: 'visualizer',
        rosMasterUri: configuration.ros1?.masterUri ?? 'http://localhost:11311/',
        hostname: RosNode.GetRosHostname(getEnvVar, getHostname, getNetworkInterfaces),
        pid: getPid(),
        httpServer: new HttpServerNodejs(),
        tcpSocketCreate: TcpSocketNode.Create,
      });

    await ros.start();

    const subscription = ros.subscribe({
        topic: '/state',
        dataType: 'std_msgs/String',
    });

    subscription.on('message', (msg: any, data: any, publisher: any) => {
        const strMsg = msg as { data: string };
        const parsed = JSON.parse(strMsg.data);

        const webview = fetchWebview();
        webview.postMessage({ command: 'visualize', data: parsed});
    });

    console.log('Listening for ROS1 communication...');

    return ros;
};