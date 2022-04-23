package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.wrapper;

import java.security.Permission;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception.EarlyExitException;

public class StopExitSecurityManager extends SecurityManager {
	private SecurityManager previousManager = System.getSecurityManager();

	public void checkPermission(Permission perm) {
	}

	public void checkExit(int status) {
		super.checkExit(status);
		throw new EarlyExitException("Exited with code: " + status, status);
	}

	public SecurityManager getPreviousMgr() {
		return previousManager;
	}
}