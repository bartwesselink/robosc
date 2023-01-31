import { Component, Input, OnInit } from '@angular/core';
import { DomSanitizer, SafeHtml } from '@angular/platform-browser';
import Prism from 'prismjs'

@Component({
  selector: 'app-code',
  templateUrl: './code.component.html',
  styleUrls: ['./code.component.scss']
})
export class CodeComponent implements OnInit {
  @Input('current') set stepSetter(value: number) {
    this.step = value;
    this.updateCode();
  }
  @Input('code') set codeSetter(value: string) {
    this.code = value;
    this.updateCode();
  }
  private code: string;
  private step: number = -1;
  public converted: SafeHtml;

  constructor(private domSanitizer: DomSanitizer) { }

  ngOnInit(): void {
  }

  updateCode() {
    const parsed = Prism.highlight(this.code, rscd, 'javascript');

    // Style gaps
    const styledGaps = parsed.replace(/(\.\.\.)/g, '<span class="gap">$1</span>');

    // Style highlight
    const styledHighlights = styledGaps
        .replace(/_highlight([0-9]+)_ /g, (match, number) => `<span class="highlight ${+number + 1 < this.step ? 'highlight-visible' : ''} ${+number + 1 === this.step ? 'highlight-animate' : ''}">`)
        .replace(/ _\/highlight_/g, '</span>');

    this.converted = this.domSanitizer.bypassSecurityTrustHtml(styledHighlights);
  }
}

const rscd = {
  'property': {
      pattern: /(^|[^\\])"(?:\\.|[^\\"\r\n])*"(?=\s*:)/,
      lookbehind: true,
      greedy: true
  },
  'string': {
      pattern: /(^|[^\\])"(?:\\.|[^\\"\r\n])*"(?!\s*:)/,
      lookbehind: true,
      greedy: true
  },
  'comment': {
      pattern: /\/\/.*|\/\*[\s\S]*?(?:\*\/|$)/,
      greedy: true
  },
  'number': /-?\b\d+(?:\.\d+)?(?:e[+-]?\d+)?\b/i,
  'punctuation': /[{}[\],]/,
  'operator': /:/,
  'keyword': /\b(cancel|use|do|integer|none|robot|feedback|action|from|links|state|if|needs|incoming|double|disables|enum|provide|true|object|outgoing|request|import|string|error|interface|default|library|goto|array|datatype|and|value|on|or|initial|marked|false|requirement|message|transition|with|component|boolean|service|response|variable|behaviour|to)\b/,
  'boolean': /\b(?:false|true)\b/,
  'null': {
      pattern: /\bnull\b/,
      alias: 'keyword'
  }
};
