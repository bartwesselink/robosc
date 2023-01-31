import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class FigureService {
  private no = 0;

  constructor() { }

  obtainNumber() {
    return ++this.no;
  }
}
