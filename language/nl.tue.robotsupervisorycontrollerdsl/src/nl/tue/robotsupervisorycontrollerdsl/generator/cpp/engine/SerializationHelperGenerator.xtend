package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine

import javax.inject.Singleton

@Singleton
class SerializationHelperGenerator {
	def generateSerializeVectorFunction()'''
std::string serialize_json_vector(const std::vector<std::string>& list) {
    std::stringstream output;
    output << "[";
    
    bool first = true;

    for (const auto& value : list) {
        if (!first) {
            output << ", ";
        }
        
        first = false;

        output << "\"" << value << "\"";
    }
    
    output << "]";   
    
    return output.str();
}
	'''
}