import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConceptsRequirementsComponent } from './concepts-requirements.component';

describe('ConceptsRequirementsComponent', () => {
  let component: ConceptsRequirementsComponent;
  let fixture: ComponentFixture<ConceptsRequirementsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConceptsRequirementsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConceptsRequirementsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
