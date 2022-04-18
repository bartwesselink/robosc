import * as vscode from 'vscode';
import * as path from 'path';
import { LanguageClient, LanguageClientOptions, ServerOptions, Trace } from 'vscode-languageclient/node';

export function activate(context: vscode.ExtensionContext) {
    const executable = process.platform === 'win32' ? 'nl.tue.robotsupervisorycontrollerdsl.ide.bat' : 'nl.tue.robotsupervisorycontrollerdsl.ide';
    const languageServerPath =  path.join('server', 'nl.tue.robotsupervisorycontrollerdsl.ide', 'bin', executable);
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
        documentSelector: ['rscd'],
        synchronize: {
            fileEvents: vscode.workspace.createFileSystemWatcher('**/*.*')
        }
    };

    const languageClient = new LanguageClient('Xtext Server', serverOptions, clientOptions);
    languageClient.trace = Trace.Verbose;
    languageClient.start();

    return languageClient;
}