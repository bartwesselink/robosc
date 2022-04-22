package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import javax.inject.Inject;

import org.eclipse.escet.common.app.framework.Application;
import org.eclipse.escet.common.app.framework.output.IOutputComponent;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.EarlyExitException;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper.CodeFlowControl;

public abstract class AbstractCifTool<T extends Application<IOutputComponent>> {
	@Inject CodeFlowControl codeFlowControl;
	
	protected boolean execute(String[] args) {
		this.captureOutput();

		System.err.printf("Executing command %s\n", this.getClass().getName());

		T app;
		try {
			app = this.getApplication().getDeclaredConstructor().newInstance();

			int exitCode = app.run(args);
			
			System.err.printf("Executed CIF-command %s\n", this.getClass().getName());
			System.err.printf("Received exit code: %d\n", exitCode);
			
			return exitCode == 0;
		} catch (EarlyExitException e) {
			// Ignore and continue
		} catch (Exception e) {
			e.printStackTrace();
			
			return false;
		}
		
		return true;
	}
	
	private void captureOutput() {
		this.codeFlowControl.disableSystemExit();
	}
	
	abstract Class<T> getApplication();
}
