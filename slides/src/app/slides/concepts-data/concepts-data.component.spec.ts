import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConceptsDataComponent } from './concepts-data.component';

describe('ConceptsDataComponent', () => {
  let component: ConceptsDataComponent;
  let fixture: ComponentFixture<ConceptsDataComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConceptsDataComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConceptsDataComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
