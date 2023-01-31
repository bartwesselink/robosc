import { Component, Input, OnInit } from '@angular/core';

@Component({
  selector: 'app-section-overview-item',
  templateUrl: './section-overview-item.component.html',
  styleUrls: ['./section-overview-item.component.scss']
})
export class SectionOverviewItemComponent implements OnInit {
  @Input() public active = false;
  public animate = false;

  constructor() { }

  ngOnInit(): void {
    setTimeout(() => {
      this.animate = this.active;
    });
  }

}
