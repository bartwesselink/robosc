import { Component, Input, OnInit } from '@angular/core';

@Component({
  selector: 'app-title',
  templateUrl: './title.component.html',
  styleUrls: ['./title.component.scss']
})
export class TitleComponent implements OnInit {
  @Input() public label: string;
  @Input() public subTitle?: string;
  @Input() public active = false;
  @Input() public small = false;

  constructor() { }

  ngOnInit(): void {
  }


}
