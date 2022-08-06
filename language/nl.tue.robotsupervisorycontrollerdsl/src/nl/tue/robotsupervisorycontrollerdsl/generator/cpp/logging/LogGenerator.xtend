package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging

import javax.inject.Singleton

@Singleton
class LogGenerator {
	def compileImports()'''
	#include <iostream>
	#include <fstream>
	'''
	
	def compileFields()'''
	std::ofstream logging_out_file;
	std::ofstream logging_in_file;
	'''
	
	def compileFunctions()'''
	std::string logging_time_human() {
		std::time_t rawtime;
		char buffer[80];
		
		std::time (&rawtime);
		auto local = std::localtime(&rawtime);
		
		std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local);
		std::string output(buffer);
		
		return output;
	}

	std::string logging_time_machine() {
		std::time_t rawtime;
		char buffer[80];
		
		std::time (&rawtime);
		auto local = std::localtime(&rawtime);
		
		std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", local);
		std::string output(buffer);
		
		return output;
	}
	
	void start_logging() {
		logging_out_file.open("node_out_" + logging_time_machine() + ".log");
		logging_in_file.open("node_in_" + logging_time_machine() + ".log");
	}
	
	void write_to_incoming_log(std::string value) {
	   logging_in_file << "[" << logging_time_human() << "] "<< value << std::endl;
	}
		
	void write_to_outgoing_log(std::string value) {
	   logging_out_file << "[" << logging_time_human() << "] "<< value << std::endl;
	}
	'''
	
	def compileInitialization()'''
	this->start_logging();
	'''
	
	def compileDestruction()'''
	logging_in_file.close();
	logging_out_file.close();
	'''
}