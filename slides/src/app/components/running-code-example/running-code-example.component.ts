import { Component, Input, OnInit } from '@angular/core';

@Component({
  selector: 'app-running-code-example',
  templateUrl: './running-code-example.component.html',
  styleUrls: ['./running-code-example.component.scss']
})
export class RunningCodeExampleComponent implements OnInit {
  @Input() public code: string;
  @Input() public start: number;
  @Input() public end: number;
  @Input() public current: number;
  @Input() public text = true;

  constructor() { }

  ngOnInit(): void {
  }

}
