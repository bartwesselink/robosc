package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper;

import java.security.Permission;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.EarlyExitException;

public class StopExitSecurityManager extends SecurityManager {
	private SecurityManager prevMgr = System.getSecurityManager();

	public void checkPermission(Permission perm) {
	}

	public void checkExit(int status) {
		super.checkExit(status);
		throw new EarlyExitException("Exited code"); // This throws an exception if an exit is called.
	}

	public SecurityManager getPreviousMgr() {
		return prevMgr;
	}
}