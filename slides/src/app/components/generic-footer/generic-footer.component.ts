import { Component, Input, OnInit } from '@angular/core';

@Component({
  selector: 'app-generic-footer',
  templateUrl: './generic-footer.component.html',
  styleUrls: ['./generic-footer.component.scss']
})
export class GenericFooterComponent implements OnInit {
  @Input() public page = 1;
  @Input() public section = false;

  constructor() { }

  ngOnInit(): void {
  }

}
