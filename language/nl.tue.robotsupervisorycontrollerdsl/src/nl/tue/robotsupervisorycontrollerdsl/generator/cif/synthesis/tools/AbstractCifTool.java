package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.io.PrintStream;

import javax.inject.Inject;

import org.eclipse.escet.common.app.framework.Application;
import org.eclipse.escet.common.app.framework.output.IOutputComponent;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.CIFException;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.EarlyExitException;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper.CodeFlowControl;

public abstract class AbstractCifTool<T extends Application<IOutputComponent>> {
	@Inject CodeFlowControl codeFlowControl;
	private OutputStream capturer;
	private PrintStream previousOut;

	protected boolean execute(String[] args) {
		this.captureOutput();

		System.out.printf("Executing command %s\n", this.getClass().getName());

		T app;
		try {
			app = this.getApplication().getDeclaredConstructor().newInstance();

			app.run(args);
			
			// This code is not reachable, as CIF will terminate
			return false;
		} catch (EarlyExitException e) {
			this.restoreOutput();
			int exitCode = e.getStatusCode();
			
			boolean success = exitCode == 0;
			
			if (!success) {
				String output = this.capturer.toString();
				
				throw new CIFException("CIF synthesis failed: " + output);
			}
			
			return success;
		} catch (Exception e) {
			this.restoreOutput();
			e.printStackTrace();
			
			return false;
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
