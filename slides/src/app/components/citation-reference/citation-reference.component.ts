import { Component, Input, OnInit } from '@angular/core';
import { CitationComponent } from '../citation/citation.component';

@Component({
  selector: 'app-citation-reference',
  templateUrl: './citation-reference.component.html',
  styleUrls: ['./citation-reference.component.scss']
})
export class CitationReferenceComponent implements OnInit {
  @Input() public citation: CitationComponent;

  constructor() { }

  ngOnInit(): void {
  }

}
