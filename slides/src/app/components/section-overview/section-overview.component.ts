import { Component, Input, OnInit } from '@angular/core';

@Component({
  selector: 'app-section-overview',
  templateUrl: './section-overview.component.html',
  styleUrls: ['./section-overview.component.scss']
})
export class SectionOverviewComponent implements OnInit {
  @Input() public current = 0;
  constructor() { }

  ngOnInit(): void {

  }

}
