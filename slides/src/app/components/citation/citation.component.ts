import { Component, Input, OnInit } from '@angular/core';
import { CitationService } from 'src/app/services/citation.service';

@Component({
  selector: 'app-citation',
  templateUrl: './citation.component.html',
  styleUrls: ['./citation.component.scss']
})
export class CitationComponent implements OnInit {
  public number: number = this.citationService.obtainNumber();
  @Input() public text: string;

  constructor(private citationService: CitationService) { }

  ngOnInit(): void {
  }

}
