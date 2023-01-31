import { TestBed } from '@angular/core/testing';

import { CitationService } from './citation.service';

describe('CitationService', () => {
  let service: CitationService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(CitationService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
