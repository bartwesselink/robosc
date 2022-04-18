package nl.tue.robotsupervisorycontrollerdsl.mwe2

import static extension org.eclipse.xtext.GrammarUtil.*
import static extension org.eclipse.xtext.xtext.generator.web.RegexpExtensions.*
import org.eclipse.xtext.xtext.generator.AbstractXtextGeneratorFragment
import org.eclipse.xtend.lib.annotations.Accessors
import javax.inject.Inject
import org.eclipse.xtext.xtext.generator.model.FileAccessFactory
import java.util.regex.Pattern

class TextMateKeywordFragment extends AbstractXtextGeneratorFragment {
	@Inject FileAccessFactory fileAccessFactory

	@Accessors(PUBLIC_SETTER)
	String name

	@Accessors(PUBLIC_SETTER)
	String scope

	@Accessors(PUBLIC_SETTER)
	String id
	
	override generate() {
		val file = fileAccessFactory.createTextFile("/textmate-gen/" + id + ".tmLanguage.json")
		file.content = '''
			{
				"name": "«name»",
			    "scopeName": "«scope»",
			    "fileTypes": [«FOR ext: language.fileExtensions SEPARATOR ","»"«ext»"«ENDFOR»],
			    "repository": {
			        "general": {
			            "patterns": [
			                {
			                    "include": "#linecomment"
			                },
			                {
			                    "include": "#blockcomment"
			                },
			                {
			                    "include": "#keyword"
			                }
			            ]
			        },
			        "linecomment": {
			            "name": "comment.line.double-dash.«id»",
			            "begin": "(^[ \\t]+)?(?=//)",
						"end": "(?=$)"
			        },
			        "blockcomment": {
			            "name": "comment.block.«id»",
			            "begin": "/\\*(\\*)?(?!/)",
						"end": "\\*/"
			        },
			        "keyword": {
			            "name": "keyword.control.«id»",
			            "match": "\\b(«keywords»)\\b"
			        }
			    },
			    "patterns": [
			        {
			            "include": "#general"
			        }
			    ]
			}
		'''
		file.writeTo(projectConfig.genericIde.root)
	}
	
	// source: https://github.com/itemis/xtext-generator-vscode/blob/master/src/main/java/com/itemis/xtext/generator/vscode/VSCodeExtensionFragment.xtend
	def protected String keywords() {
		val allKeywords = grammar.allKeywords
		val wordKeywords = newArrayList
		val nonWordKeywords = newArrayList

		val keywordsFilterPattern = Pattern.compile('\\w+')
		val wordKeywordPattern = Pattern.compile('\\w(.*\\w)?')

		allKeywords.filter[keywordsFilterPattern.matcher(it).matches].forEach[
			if (wordKeywordPattern.matcher(it).matches)
				wordKeywords += it
			else
				nonWordKeywords += it
		]
		
		val result = (wordKeywords + nonWordKeywords).map[it.toRegexpString(false)].join('|')

		return result
	}
	
}