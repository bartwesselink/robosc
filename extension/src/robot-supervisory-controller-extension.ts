import * as vscode from 'vscode';
import * as path from 'path';
import { LanguageClient, LanguageClientOptions, ServerOptions, Trace } from 'vscode-languageclient/node';
import { startRos1Listener } from './ros-1-listener';
import { startRos2Listener } from './ros-2-listener';
import { Configuration, Middleware } from './models';

export function activate(context: vscode.ExtensionContext) {
    const executable = process.platform === 'win32' ? 'nl.tue.robotsupervisorycontrollerdsl.ide.bat' : 'nl.tue.robotsupervisorycontrollerdsl.ide';
    const languageServerPath = path.join('server', 'nl.tue.robotsupervisorycontrollerdsl.ide', 'bin', executable);
    const script = context.asAbsolutePath(languageServerPath);

    const serverOptions: ServerOptions = {
        run: {
            command: script,
        },
        debug: {
            command: script,
        }
    };

    let clientOptions: LanguageClientOptions = {
        documentSelector: ['rsc'],
        synchronize: {
            fileEvents: vscode.workspace.createFileSystemWatcher('**/*.*')
        }
    };

    const languageClient = new LanguageClient('Xtext Server', serverOptions, clientOptions);
    languageClient.trace = Trace.Verbose;
    languageClient.start();

    let currentPanel: vscode.WebviewPanel|undefined;

    // Add visualization command
    context.subscriptions.push(
        vscode.commands.registerCommand('rsc.visualizeController', async () => {
            const column = vscode.window.activeTextEditor
                ? vscode.window.activeTextEditor.viewColumn
                : undefined;

            // If we already have a panel, show it.
            if (currentPanel) {
                currentPanel.reveal(column);
                return;
            }

            // Create and show a new webview
            currentPanel = vscode.window.createWebviewPanel(
                'rscControllerVisualization',
                'Controller Visualization',
                column || vscode.ViewColumn.One,
                {
                    enableScripts: true,
                    localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'visualization')],
                    retainContextWhenHidden: true,
                }
            );

            currentPanel.onDidDispose(
                () => {
                    currentPanel = undefined;
                },
                null,
                context.subscriptions
              );

            const configuration = vscode.workspace.getConfiguration('rsc') as unknown as Configuration;
            let currentDisposeFunction: () => Promise<void>;

            currentPanel.webview.onDidReceiveMessage(async (message) => {
                if (message.command !== 'middleware') return;

                if (currentDisposeFunction) {
                    await currentDisposeFunction();
                }

                switch (message.data) {
                    case Middleware.ROS1:
                        const ros1 = await startRos1Listener(() => currentPanel!!.webview, configuration);
                        currentDisposeFunction = async () => await ros1.shutdown();

                        break;
                    case Middleware.ROS2:
                        const ros2 = await startRos2Listener(() => currentPanel!!.webview, configuration);
                        currentDisposeFunction = async () => await ros2.shutdown()
                        break;
                }
            }, undefined, context.subscriptions);

            currentPanel.webview.html = getWebviewContent(currentPanel.webview, context);

            setTimeout(() => {
                currentPanel?.webview.postMessage({
                    command: 'settings',
                    data: configuration,
                })
            });

            context.subscriptions.push(vscode.Disposable.from({
                dispose: async () => {
                    if (currentDisposeFunction) {
                        await currentDisposeFunction();
                    }
                },
            }));
        })
    );

    return languageClient;
}

function getWebviewContent(webview: vscode.Webview, context: vscode.ExtensionContext) {
    const scriptPath = webview.asWebviewUri(
        vscode.Uri.joinPath(context.extensionUri, 'visualization', 'dist', 'main.js')
    );

    return `<!doctype html>
    <html>
        <head>
            <meta http-equiv="Content-Security-Policy" content="default-src 'none';  img-src ${webview.cspSource} https:; script-src 'unsafe-eval' ${webview.cspSource}; style-src 'unsafe-inline' ${webview.cspSource};">
        </head>
        <body>
            <div id="root"></div>
            <div class="middleware-switcher">
                <div class="middleware-option">ROS1</div>
                <div class="middleware-option">ROS2</div>
            </div>
            <script src="${scriptPath}" type="text/javascript"></script>
        </body>
    </html>`;
}