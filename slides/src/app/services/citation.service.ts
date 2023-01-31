import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class CitationService {
  private no = 0;

  constructor() { }

  obtainNumber() {
    return ++this.no;
  }
}
