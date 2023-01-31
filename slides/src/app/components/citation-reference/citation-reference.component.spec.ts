import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CitationReferenceComponent } from './citation-reference.component';

describe('CitationReferenceComponent', () => {
  let component: CitationReferenceComponent;
  let fixture: ComponentFixture<CitationReferenceComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ CitationReferenceComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CitationReferenceComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
