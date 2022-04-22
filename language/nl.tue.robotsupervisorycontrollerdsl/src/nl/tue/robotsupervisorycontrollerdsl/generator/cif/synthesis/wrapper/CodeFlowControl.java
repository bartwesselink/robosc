package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper;

import javax.inject.Singleton;

@Singleton
public class CodeFlowControl {
	public void disableSystemExit() {
		SecurityManager securityManager = new StopExitSecurityManager();
		System.setSecurityManager(securityManager);
	}

	public void enableSystemExit() {
		SecurityManager manager = System.getSecurityManager();

		if ((manager != null) && (manager instanceof StopExitSecurityManager)) {
			StopExitSecurityManager smgr = (StopExitSecurityManager) manager;
			System.setSecurityManager(smgr.getPreviousMgr());
		} else {
			System.setSecurityManager(null);
		}
	}
}
