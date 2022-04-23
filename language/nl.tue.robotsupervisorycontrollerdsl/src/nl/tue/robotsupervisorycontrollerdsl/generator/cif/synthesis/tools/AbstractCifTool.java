package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.io.PrintStream;

import javax.inject.Inject;

import org.eclipse.escet.common.app.framework.Application;
import org.eclipse.escet.common.app.framework.output.IOutputComponent;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.EarlyExitException;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper.CodeFlowControl;

public abstract class AbstractCifTool<T extends Application<IOutputComponent>> {
	@Inject CodeFlowControl codeFlowControl;
	private OutputStream capturer;
	private PrintStream previousOut;

	protected boolean execute(String[] args) {
		this.captureOutput();

		System.err.printf("Executing command %s\n", this.getClass().getName());

		T app;
		try {
			app = this.getApplication().getDeclaredConstructor().newInstance();

			int exitCode = app.run(args);
			
			System.err.printf("Executed CIF-command %s\n", this.getClass().getName());
			System.err.printf("Received exit code: %d\n", exitCode);
			
			// This code is not reachable, as CIF will terminate
			return false;
		} catch (EarlyExitException e) {
			int exitCode = e.getStatusCode();
			
			return exitCode == 0;
		} catch (Exception e) {
			e.printStackTrace();
			
			return false;
		} finally {
			this.restoreOutput();
		}
	}
	
	private void captureOutput() {
		this.codeFlowControl.disableSystemExit();

		capturer = new ByteArrayOutputStream();
		previousOut = System.err;
		
		PrintStream printStream = new PrintStream(capturer);
		System.setErr(printStream);
	}
	
	private void restoreOutput() {
		this.codeFlowControl.enableSystemExit();
		
		capturer = null;
		System.setErr(previousOut);		
	}
	
	abstract Class<T> getApplication();
}
