import * as vscode from 'vscode';
import * as path from 'path';
import { LanguageClient, LanguageClientOptions, ServerOptions } from 'vscode-languageclient/node';

export function activate(context: vscode.ExtensionContext) {
    const executable = process.platform === 'win32' ? 'nl.tue.robotsupervisorycontrollerdsl.ide.bat' : 'nl.tue.robotsupervisorycontrollerdsl.ide';
    const languageServerPath =  path.join('server', 'nl.tue.robotsupervisorycontrollerdsl.ide', 'bin', executable);
    const serverLauncher = context.asAbsolutePath(languageServerPath);
    const serverOptions: ServerOptions = {
        run: {
            command: serverLauncher,
            args: ['-trace']
        },
        debug: {
            command: serverLauncher,
            args: ['-trace']
        }
    };
    
    let clientOptions: LanguageClientOptions = {
        documentSelector: ['rscd'],
        synchronize: {
            fileEvents: vscode.workspace.createFileSystemWatcher('**/*.*')
        }
    };

    const languageClient = new LanguageClient('rscdLanguageClient', 'Robot Supervisory Controller DSL Language Server', serverOptions, clientOptions);
    languageClient.start();

    return languageClient;
}
